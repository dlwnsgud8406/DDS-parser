# 회식 메뉴 정하기 백엔드 Copilot Instructions

## Project Overview

본 프로젝트는 익명의 참여자들이 특정 회식 이벤트에 대한 메뉴 후보를 제안하고, 투표하며, 그 결과를 확인할 수 있는 FastAPI 기반의 백엔드 시스템입니다.

호스트가 회식 이벤트를 생성하고, 참여자들이 메뉴 후보를 제안할 수 있으며, 각 참여자는 자신이 선호하는 메뉴에 투표할 수 있습니다. 최종적으로 가장 많은 투표를 받은 메뉴가 회식 메뉴로 선정됩니다.

사용자는 회식 메뉴 후보를 제안하고, 투표를 통해 선호하는 메뉴를 선택할 수 있으며, 최종적으로 가장 많은 투표를 받은 메뉴가 회식 메뉴로 선정됩니다.

Layered architecture로 구성되어 있으며, 각 계층은 다음과 같은 역할을 수행합니다:

### Root Folders

- `app/`: FastAPI 앱 (routers, models, schemas)
- `test/`: Integration tests and test infrastructure

### Core Architecture (`app/` folder)

- `app/api/`: FastAPI Routers (프레젠테이션)
- `app/service/`: 비즈니스 로직 (애플리케이션)
- `app/repository/`: DB 접근 계층 (MongoDB 사용)
- `app/models/`: Pydantic Document 모델 (도메인)
- `app/schemas/`: Pydantic DTO (전송 객체)

### 가상 환경

본 프로젝트는 Python 가상 환경으로 `.venv` 디렉토리를 사용합니다. 모든 패키지 설치 및 실행은 `.venv` 환경에서 이루어져야 합니다.

```
source .venv/bin/activate
```

### Finding Related Code

1. **먼저 시맨틱 검색**: 일반적인 개념을 파일 검색으로 찾으세요.
2. **정확한 문자열은 grep 사용**: 에러 메시지나 특정 함수명을 grep으로 찾으세요.
3. **import 따라가기**: 문제가 되는 모듈을 import하는 파일을 확인하세요.
4. **테스트 파일 확인**: 사용 패턴과 기대 동작을 자주 보여줍니다.