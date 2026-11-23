"""SMART-LOGISTICS 웹 관제 서버 (E).

실행:  uvicorn app.main:app --host 0.0.0.0 --port 8000      (web_control/ 에서)
환경변수:
  RUN_MODE   MOCK (기본) — A 테스트 대역 사용. SIM/REAL 연동은 P-003 결정 후 추가
  DB_PATH    SQLite 경로 (기본 web_control/data/web_control.db)
  MOCK_SPEED MOCK 로봇 이동 배속 (기본 4)
  START_WAYPOINT  A register_jobs 의 start_waypoint_id (기본 HOME)       [미정: A가 정할지 E가 넣을지]
  COST_REVISION   A register_jobs 의 cost_revision (기본 mock-table)    [미정: C 비용표 버전을 누가 넣을지]

[초안-팀합의필요] 아래 API 경로·필드 이름은 A·E 합의 전 초안이다.
"""
import asyncio
import json
import os
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .db import Database
from .mission_client import MissionClient, MissionInputError, MissionRecoveryPending, MissionTimeout
from .requests_service import RequestRejected, RequestService

BASE_DIR = Path(__file__).resolve().parent.parent


def _waypoint_file() -> dict:
    return json.loads((BASE_DIR / "config" / "waypoints.json").read_text(encoding="utf-8"))


def load_waypoints() -> dict:
    return {w["id"]: w for w in _waypoint_file()["waypoints"]}


class JobIn(BaseModel):
    pickup: str
    dropoff: str


class HandoffIn(BaseModel):
    # A는 빈 식별자를 입력 오류로 처리한다 → E가 먼저 막는다 (422)
    mission_id: str = Field(min_length=1, pattern=r"\S")
    step_id: str = Field(min_length=1, pattern=r"\S")


class MissionRunIn(BaseModel):
    mission_id: str = Field(min_length=1, pattern=r"\S")


class FaultIn(BaseModel):
    drop_next_ack: bool = False
    not_ready: bool = False
    timeout_next: bool = False


class RecoveryIn(BaseModel):
    required: bool


def create_app(db_path: str | Path | None = None, client: MissionClient | None = None,
               run_mode: str | None = None, auto_sync: bool | None = None) -> FastAPI:
    """테스트에서 임시 DB·A 대역을 넣어 만들 수 있도록 팩토리로 둔다."""
    run_mode = (run_mode or os.getenv("RUN_MODE", "MOCK")).upper()
    waypoints = load_waypoints()
    waypoint_source = _waypoint_file().get("source", "")
    db_path = db_path or os.getenv("DB_PATH", BASE_DIR / "data" / "web_control.db")
    if client is None:
        if run_mode != "MOCK":
            raise RuntimeError("SIM/REAL 연동(MissionClient 실제 구현)은 P-003 결정 전이라 아직 없습니다. RUN_MODE=MOCK 으로 실행하세요")
        from .mock_mission import MockMissionManager
        client = MockMissionManager(waypoints, speed_scale=float(os.getenv("MOCK_SPEED", "4")))

    if auto_sync is None:
        auto_sync = os.getenv("AUTO_SYNC", "1") != "0"
    clients: set[WebSocket] = set()
    ctx: dict = {"last_state": None}

    async def snapshot() -> dict:
        try:
            a_state = await client.snapshot()
            a_ok = True
        except Exception as e:  # A 상태를 못 받으면 화면에 '연결 끊김'을 표시 (E가 상태를 추측하지 않음)
            a_state, a_ok = {"error": f"{type(e).__name__}: {e}"}, False
        return {
            "run_mode": run_mode,
            "a_connected": a_ok,
            "a_state": a_state,
            "job_requests": svc.list_job_requests(50),
            "handoffs": svc.list_handoffs(50),
        }

    async def broadcast():
        snap = await snapshot()
        text = json.dumps(snap, ensure_ascii=False, default=str)
        ctx["last_state"] = text
        for ws in list(clients):
            try:
                await ws.send_text(text)
            except Exception:
                clients.discard(ws)

    start_waypoint = os.getenv("START_WAYPOINT", "HOME")
    cost_revision = os.getenv("COST_REVISION", "mock-table")
    db = Database(db_path)
    svc = RequestService(db, client, on_change=broadcast,
                         start_waypoint_id=start_waypoint, cost_revision=cost_revision)

    async def poll_a():
        """A 상태가 바뀌면 방송 (전송 방식 확정 전 MOCK에서는 폴링)."""
        while True:
            snap = await snapshot()
            if auto_sync and snap["a_connected"]:
                try:
                    if await svc.sync_with_a(snap["a_state"]):   # A와 어긋난 기록을 같은 ID로 다시 보내 맞춤
                        snap = await snapshot()
                except Exception:
                    pass   # 동기화 실패는 다음 주기에 다시 시도 (기록은 열린 상태 그대로)
            text = json.dumps(snap, ensure_ascii=False, default=str)
            if text != ctx["last_state"]:
                ctx["last_state"] = text
                for ws in list(clients):
                    try:
                        await ws.send_text(text)
                    except Exception:
                        clients.discard(ws)
            await asyncio.sleep(0.2)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        if hasattr(client, "start"):
            await client.start()
        app.state.recovered = await svc.recover()   # 재시작 시 결과 미확인 요청 재전송
        poller = asyncio.create_task(poll_a())
        yield
        poller.cancel()
        if hasattr(client, "stop"):
            await client.stop()
        db.close()

    app = FastAPI(title="SMART-LOGISTICS Web Control (E)", lifespan=lifespan)
    app.state.svc, app.state.client, app.state.db = svc, client, db

    # ------------------------------------------------------------ 조회
    @app.get("/api/config")
    def get_config():
        return {"run_mode": run_mode, "waypoints": list(waypoints.values()),
                "waypoint_source": waypoint_source, "start_waypoint": start_waypoint, "cost_revision": cost_revision}

    @app.get("/api/state")
    async def get_state():
        return await snapshot()

    @app.get("/api/recovery")
    def get_recovery():
        return getattr(app.state, "recovered", None)

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        clients.add(ws)
        await ws.send_text(json.dumps(await snapshot(), ensure_ascii=False, default=str))
        try:
            while True:
                await ws.receive_text()
        except WebSocketDisconnect:
            pass
        finally:
            clients.discard(ws)

    # ------------------------------------------------------------ Job 요청 (E → A)
    @app.post("/api/job-requests", status_code=201)
    async def create_job_request(body: JobIn):
        if body.pickup not in waypoints or body.dropoff not in waypoints:
            raise HTTPException(422, f"등록되지 않은 지점입니다 (가능: {', '.join(waypoints)})")
        if body.pickup == body.dropoff:
            raise HTTPException(422, "수거 지점과 전달 지점이 같습니다")
        return await svc.request_job(body.pickup, body.dropoff)

    @app.post("/api/job-requests/{request_id}/retry")
    async def retry_job_request(request_id: str):
        try:
            return await svc.retry_job(request_id)
        except KeyError:
            raise HTTPException(404, "요청 기록이 없습니다")

    @app.get("/api/job-requests")
    def list_job_requests():
        return svc.list_job_requests()

    # ------------------------------------------------------------ 인계 요청 (E → A)
    @app.post("/api/mission/run")
    async def request_mission_run(body: MissionRunIn):
        """미션 실행 요청 (INTERFACES §11.1). 허용 여부는 A가 결정한다. 결과를 모르면 504 → 같은 요청을 다시 보내면 된다."""
        try:
            result = await client.request_mission_run(body.mission_id)
        except MissionInputError as e:
            raise HTTPException(422, f"A 입력 오류: {e}")
        except MissionRecoveryPending as e:
            raise HTTPException(503, f"A 복구 확인 중: {e}")
        except MissionTimeout as e:
            raise HTTPException(504, f"결과 미확인 — 다시 요청하세요 ({e})")
        await broadcast()
        return {"mission_id": body.mission_id, **result}

    @app.post("/api/handoffs", status_code=201)
    async def create_handoff(body: HandoffIn):
        try:
            return await svc.request_handoff(body.mission_id, body.step_id)
        except RequestRejected as e:
            raise HTTPException(409, str(e))

    @app.post("/api/handoffs/{event_id}/retry")
    async def retry_handoff(event_id: str):
        try:
            return await svc.retry_handoff(event_id)
        except KeyError:
            raise HTTPException(404, "인계 기록이 없습니다")

    @app.get("/api/handoffs")
    def list_handoffs():
        return svc.list_handoffs()

    # ------------------------------------------------------------ MOCK 결함 주입 (시연·테스트용)
    if run_mode == "MOCK":
        @app.post("/api/mock/faults")
        async def set_faults(body: FaultIn):
            if not hasattr(client, "drop_next_ack"):
                raise HTTPException(400, "결함 주입을 지원하지 않는 대역입니다")
            client.drop_next_ack, client.timeout_next = body.drop_next_ack, body.timeout_next
            client.ready = not body.not_ready
            return body

        @app.post("/api/mock/recovery")
        async def set_recovery(body: RecoveryIn):
            """A 재시작 후 복구 확인 중 상태를 켜고 끈다 (RecoveryPending 시연)."""
            if not hasattr(client, "recovery_required"):
                raise HTTPException(400, "복구 상태를 지원하지 않는 대역입니다")
            client.recovery_required = body.required
            await broadcast()
            return body

    app.mount("/", StaticFiles(directory=BASE_DIR / "static", html=True), name="static")
    return app


app = create_app() if os.getenv("WEB_CONTROL_NO_APP") is None else None
