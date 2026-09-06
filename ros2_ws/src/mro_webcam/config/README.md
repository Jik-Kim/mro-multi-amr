호모그래피 json 은 **찍은 맵의 좌표계에 묶인다.** 아레나·웹캠·맵 중 하나라도 바뀌면 재캘리브레이션:

상세 GUI 사용법은 [`CALIBRATION_GUIDE.md`](CALIBRATION_GUIDE.md)를 참고한다.

```bash
ros2 run mro_webcam calibrate_homography calib --cam <번호> --map <mro_bringup>/maps/auto_map.yaml --out homography_cam1.json
ros2 run mro_webcam calibrate_homography test  --cam <번호> --out homography_cam1.json
```
`homography_cam1.json` 은 Kn1_mini auto_map 기준 값(검증 오차 0.11~0.27 m)이며 새 아레나에서는 **무효**다. cam2 는 비어 있다.

## WebCam2 마커 캘리브레이션 이력

`nodes/calibrate_cam2_markers.py`는 GUI에서 선택한 이미지 또는 라이브 카메라 프레임을
`config/WebCam2/`에 다음 세 파일의 묶음으로 저장한다.

- `reference.png`: 보정에 사용한 기준 이미지
- `points.png`: 마커 순서와 실좌표가 표시된 이미지
- `homography_cam2.json`: 호모그래피와 재투영 오차

재보정 시 기존 폴더는 `WebCam2_YYYYMMDD_HHMMSS`로 이름을 바꿔 보존한 뒤
새 `WebCam2`를 생성한다. 같은 초에 여러 번 저장하면 `_01`, `_02` 순번을 덧붙인다.

```bash
# GUI 탭에서 라이브/불러오기 모드 선택 (기본: 불러오기)
ros2 run mro_webcam calibrate_cam2_markers
```

모드는 하나의 GUI 창에 있는 `불러오기`/라이브 탭으로 선택하며
`불러오기`가 기본 탭이다. 라이브 모드는 현재 카메라
인덱스를 GUI 콤보박스에서 고른 뒤 해당 프레임에서 시작한다. 불러오기 모드는
이미지와 JSON을 독립적으로 선택한다.

- 이미지의 민트색 3x3 px 마커는 자동 검출된다.
- JSON을 선택하면 이미지 마커와 가장 가까운 기존 픽셀 좌표의 실좌표를 연결한다.
- JSON 불러오기를 해제하면 이미지만 열고, 검출된 마커의 실좌표는 미입력 상태로 시작한다.
- 마커가 없는 일반 이미지는 빈 상태에서 직접 마커를 추가하면 된다.

실측 좌표 입력 단위는 항상 `mm`다. 좌클릭으로 마커를
추가하거나 기존 실좌표를 수정하고,
우클릭으로 마커를 삭제한다. 선택한 마커는 `마커 미세 이동` 패널의
상·하·좌·우 버튼으로 1 px씩 옮길 수 있으며 실좌표는 유지된다. `Enter` 또는 `S`로
저장하고 `Esc` 또는 `Q`로 취소한다.
