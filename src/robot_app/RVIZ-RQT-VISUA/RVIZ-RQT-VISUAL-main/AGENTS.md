# Repository Guidelines

## Project Structure & Module Organization
The repo holds two services. `backend/` hosts the FastAPI ROS bridge; code lives under `app/` with routers in `app/api/v1`, shared logic in `app/services`, helpers in `app/utils`, and tests in `backend/tests` plus `test_pointcloud_subscription.py` for manual ROS checks. `frontend/` is the Vue 3/Vite client (`src/main.js`, `src/router`, `src/composables`/`services`, `src/components`). Supporting assets sit in `img/`, runtime config in `.env`, and deployment files are the `Dockerfile` and `start.sh`.

## Build, Test, and Development Commands
Run `./start.sh local` to start FastAPI on `:8000` and Vite on `:3000`; `./start.sh docker` builds and runs the combined container. The backend loop is `cd backend && python -m venv venv && source venv/bin/activate && pip install -r requirements.txt && python -m uvicorn app.main:app --reload --host 0.0.0.0 --port 8000`. Frontend work happens with `cd frontend && npm install && npm run dev`, `npm run build` for production bundles, and `npm run preview` for static smoke tests.

## Coding Style & Naming Conventions
Python uses 4-space indents and `snake_case` modules; run `python -m black app tests`, `python -m isort app tests`, and `python -m flake8` before pushing. Vue SFCs stay PascalCase (`MapPanel.vue`), helpers prefer `kebab-case.js`, and shared tokens stay close to their components. Enforce linting with `npm run lint` and formatting with `npm run format`. Mirror ROS topic strings exactly (e.g., `/fastlio2/world_cloud`) between backend models and Pinia stores.

## Testing Guidelines
Run `pytest backend/tests` (pytest-asyncio ready) and keep fixtures centralized in `backend/tests/conftest.py`. Validate live rosbridge wiring with `python backend/test_pointcloud_subscription.py`, noting the domain ID and topics exercised. Frontend suites are not yet present; add new specs under `frontend/src/__tests__/` named `*.spec.ts` so they are Vitest-ready. Any change under `app/api` or `src/components` should include at least smoke coverage.

## Commit & Pull Request Guidelines
No Git history is bundled, so follow Conventional Commits (`feat: lidar legend`, `fix: websocket reconnect`). PRs must describe the ROS scenario tested, list validation commands (`pytest`, `npm run build`, `./start.sh docker`), link issues, and attach UI/RViz screenshots when visuals change. Call out `.env` or port adjustments explicitly.

## Configuration & Security Notes
Copy `.env`, keep it untracked, and adjust `ROS_DOMAIN_ID`, `ROSBRIDGE_PORT` (9090), `WEB_HOST`, `WEB_PORT`, and `LOG_LEVEL` per deployment. Never post real `.env` values; rely on secrets tooling. When using Docker, only bind `/tmp/.X11-unix` if 3D widgets need host X11, and prefer `start.sh docker` so ports 3000/8000/9090 stay aligned.
