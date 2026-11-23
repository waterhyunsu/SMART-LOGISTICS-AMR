"""SMART-LOGISTICS 웹 관제 서버 (E).

실행:  uvicorn app.main:app --host 0.0.0.0 --port 8000      (web_control/ 에서)
환경변수:
  RUN_MODE   MOCK (기본) — A 테스트 대역 사용. SIM/REAL 연동은 P-003 결정 후 추가
  DB_PATH    SQLite 경로 (기본 web_control/data/web_control.db)
  MOCK_SPEED MOCK 로봇 이동 배속 (기본 4)

[초안-팀합의필요] 아래 API 경로·필드 이름은 A·E 합의 전 초안이다.
"""
import asyncio
import json
import os
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .db import Database
from .mission_client import MissionClient, MissionTimeout
from .requests_service import RequestRejected, RequestService

BASE_DIR = Path(__file__).resolve().parent.parent


def load_waypoints() -> dict:
    data = json.loads((BASE_DIR / "config" / "waypoints.json").read_text(encoding="utf-8"))
    return {w["id"]: w for w in data["waypoints"]}


class JobIn(BaseModel):
    pickup: str
    dropoff: str


class HandoffIn(BaseModel):
    mission_id: str
    step_id: str


class FaultIn(BaseModel):
    drop_next_ack: bool = False
    timeout_next: bool = False


class RecoveryIn(BaseModel):
    required: bool


def create_app(db_path: str | Path | None = None, client: MissionClient | None = None,
               run_mode: str | None = None) -> FastAPI:
    """테스트에서 임시 DB·A 대역을 넣어 만들 수 있도록 팩토리로 둔다."""
    run_mode = (run_mode or os.getenv("RUN_MODE", "MOCK")).upper()
    waypoints = load_waypoints()
    db_path = db_path or os.getenv("DB_PATH", BASE_DIR / "data" / "web_control.db")
    if client is None:
        if run_mode != "MOCK":
            raise RuntimeError("SIM/REAL 연동(MissionClient 실제 구현)은 P-003 결정 전이라 아직 없습니다. RUN_MODE=MOCK 으로 실행하세요")
        from .mock_mission import MockMissionManager
        client = MockMissionManager(waypoints, speed_scale=float(os.getenv("MOCK_SPEED", "4")))

    clients: set[WebSocket] = set()
    ctx: dict = {"last_state": None}

    async def snapshot() -> dict:
        try:
            a_state = await client.get_state()
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

    db = Database(db_path)
    svc = RequestService(db, client, on_change=broadcast)

    async def poll_a():
        """A 상태가 바뀌면 방송 (전송 방식 확정 전 MOCK에서는 폴링)."""
        while True:
            snap = await snapshot()
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
        return {"run_mode": run_mode, "waypoints": list(waypoints.values())}

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

    # ------------------------------------------------------------ 미션 실행 요청 (E → A)
    @app.post("/api/mission/start")
    async def start_mission():
        try:
            res = await client.start_mission()
        except MissionTimeout as e:
            raise HTTPException(504, f"응답 미확인: {e}. 미션 상태를 확인한 뒤 다시 시도하세요")
        await broadcast()
        if res.get("outcome") != "accepted":
            raise HTTPException(409, res.get("reason") or "A가 거부했습니다")
        return res

    # ------------------------------------------------------------ 인계 요청 (E → A)
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
