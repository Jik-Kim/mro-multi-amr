호모그래피 json 은 **찍은 맵의 좌표계에 묶인다.** 아레나·웹캠·맵 중 하나라도 바뀌면 재캘리브레이션:

```bash
ros2 run mro_webcam calibrate_homography calib --cam <번호> --map <mro_bringup>/maps/auto_map.yaml --out homography_cam1.json
ros2 run mro_webcam calibrate_homography test  --cam <번호> --out homography_cam1.json
```
`homography_cam1.json` 은 Kn1_mini auto_map 기준 값(검증 오차 0.11~0.27 m)이며 새 아레나에서는 **무효**다. cam2 는 비어 있다.
