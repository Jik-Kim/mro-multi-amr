#!/usr/bin/env python3
import argparse
import json
import tempfile
from datetime import datetime
from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, simpledialog, ttk

import cv2
import numpy as np


# =========================
# 사용자 설정
# =========================
PACKAGE_DIR = Path(__file__).resolve().parents[2]
OUTPUT_DIR = PACKAGE_DIR / "config" / "WebCam2"
REFERENCE_NAME = "reference.png"
POINTS_NAME = "points.png"
CALIBRATION_NAME = "homography_cam2.json"
WINDOW_NAME = "WebCam2 calibration"
MINT_BGR = (252, 255, 0)
MARKER_PICK_RADIUS_PX = 12
INPUT_UNIT = "mm"
MM_TO_M = 0.001
MARKER_MATCH_RADIUS_PX = 15


def compute_homography(image_points, map_points):
    src = np.asarray(image_points, dtype=np.float64)
    dst = np.asarray(map_points, dtype=np.float64)

    if len(src) < 4:
        raise RuntimeError("Homography 계산에는 최소 4개의 대응점이 필요합니다.")

    # 모든 대응점을 최소제곱으로 사용
    H, _mask = cv2.findHomography(src, dst, method=0)

    if H is None:
        raise RuntimeError("Homography 계산에 실패했습니다.")

    # reprojection error
    src_h = src.reshape(-1, 1, 2)
    projected = cv2.perspectiveTransform(src_h, H).reshape(-1, 2)

    errors = np.linalg.norm(projected - dst, axis=1)
    rmse = float(np.sqrt(np.mean(errors ** 2)))

    return H, errors, rmse


class MarkerDetector:
    """이미지의 민트색 3x3 px 마커 중심을 검출한다."""

    HSV_LOWER = np.array([80, 150, 150], dtype=np.uint8)
    HSV_UPPER = np.array([100, 255, 255], dtype=np.uint8)

    @classmethod
    def detect(cls, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, cls.HSV_LOWER, cls.HSV_UPPER)
        count, _labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask, connectivity=8
        )
        points = []
        for index in range(1, count):
            area = int(stats[index, cv2.CC_STAT_AREA])
            width = int(stats[index, cv2.CC_STAT_WIDTH])
            height = int(stats[index, cv2.CC_STAT_HEIGHT])
            if 5 <= area <= 20 and width <= 6 and height <= 6:
                points.append(tuple(map(float, centroids[index])))
        return cls.sort_by_rows(points)

    @staticmethod
    def sort_by_rows(points, tolerance=8):
        rows = []
        for point in sorted(points, key=lambda item: item[1]):
            for row in rows:
                if abs(point[1] - np.mean([item[1] for item in row])) <= tolerance:
                    row.append(point)
                    break
            else:
                rows.append([point])
        rows.sort(key=lambda row: np.mean([item[1] for item in row]))
        return [point for row in rows for point in sorted(row)]


class MarkerRenderer:
    """3x3 마커와 포인트 주석을 그리고 제거한다."""

    @staticmethod
    def render_markers(image, points):
        marked = image.copy()
        height, width = marked.shape[:2]
        for u, v in points:
            u_i = int(round(u))
            v_i = int(round(v))
            marked[
                max(0, v_i - 1):min(height, v_i + 2),
                max(0, u_i - 1):min(width, u_i + 2),
            ] = MINT_BGR
        return marked

    @staticmethod
    def erase_markers(image, points):
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        height, width = mask.shape
        for u, v in points:
            u_i = int(round(u))
            v_i = int(round(v))
            cv2.rectangle(
                mask,
                (max(0, u_i - 1), max(0, v_i - 1)),
                (min(width - 1, u_i + 1), min(height - 1, v_i + 1)),
                255,
                -1,
            )
        return cv2.inpaint(image, mask, 2, cv2.INPAINT_TELEA)

    @staticmethod
    def draw_annotations(image, points, map_points):
        preview = image.copy()
        for i, ((u, v), map_point) in enumerate(
            zip(points, map_points), start=1
        ):
            center = (int(round(u)), int(round(v)))
            cv2.circle(preview, center, 6, (0, 0, 255), 1)
            cv2.putText(
                preview, f"M{i}", (center[0] + 7, center[1] - 7),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1,
                cv2.LINE_AA,
            )
            coordinate_text = "실좌표 미입력"
            if map_point is not None:
                x, y = map_point
                coordinate_text = (
                    f"({x / MM_TO_M:.1f}, {y / MM_TO_M:.1f}) mm"
                )
            cv2.putText(
                preview, coordinate_text,
                (center[0] + 7, center[1] + 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1,
                cv2.LINE_AA,
            )
        return preview


class CalibrationStore:
    """캘리브레이션 3파일의 불러오기·저장·이력 관리를 담당한다."""

    def __init__(self, output_dir):
        self.output_dir = Path(output_dir)

    def load(self, image_path=None, json_path=None):
        image_path = Path(image_path or self.output_dir / REFERENCE_NAME)
        if image_path.name == POINTS_NAME:
            raise ValueError(
                "points.png에는 포인트 주석이 포함됩니다. "
                "reference.png 또는 원본 이미지를 선택하세요."
            )
        image = cv2.imread(str(image_path))
        if image is None:
            raise FileNotFoundError(f"마커 이미지를 열 수 없습니다: {image_path}")
        detected_points = MarkerDetector.detect(image)
        points = list(detected_points)
        map_points = [None] * len(points)

        if json_path:
            json_points, json_map_points = self.load_json(json_path)
            if points:
                available = set(range(len(json_points)))
                for index, point in enumerate(points):
                    if not available:
                        break
                    nearest = min(
                        available,
                        key=lambda item: np.hypot(
                            json_points[item][0] - point[0],
                            json_points[item][1] - point[1],
                        ),
                    )
                    distance = np.hypot(
                        json_points[nearest][0] - point[0],
                        json_points[nearest][1] - point[1],
                    )
                    if distance <= MARKER_MATCH_RADIUS_PX:
                        map_points[index] = json_map_points[nearest]
                        available.remove(nearest)
            else:
                points = json_points
                map_points = json_map_points

        clean_image = MarkerRenderer.erase_markers(image, detected_points)
        return clean_image, points, map_points

    @staticmethod
    def load_json(json_path):
        json_path = Path(json_path)
        if not json_path.is_file():
            raise FileNotFoundError(f"캘리브레이션 JSON이 없습니다: {json_path}")
        with open(json_path, encoding="utf-8") as file:
            data = json.load(file)
        image_by_id = {point["id"]: point for point in data["image_points"]}
        map_by_id = {point["id"]: point for point in data["map_points"]}
        point_ids = [point["id"] for point in data["image_points"]]
        if set(image_by_id) != set(map_by_id):
            raise ValueError(
                "JSON의 image_points와 map_points ID가 일치하지 않습니다."
            )
        points = [
            (float(image_by_id[point_id]["u"]),
             float(image_by_id[point_id]["v"]))
            for point_id in point_ids
        ]
        map_points = [
            (float(map_by_id[point_id]["x"]),
             float(map_by_id[point_id]["y"]))
            for point_id in point_ids
        ]
        return points, map_points

    def archive(self, now=None):
        if not self.output_dir.exists():
            return None
        timestamp = (now or datetime.now()).strftime("%Y%m%d_%H%M%S")
        archive_dir = self.output_dir.with_name(
            f"{self.output_dir.name}_{timestamp}"
        )
        suffix = 1
        while archive_dir.exists():
            archive_dir = self.output_dir.with_name(
                f"{self.output_dir.name}_{timestamp}_{suffix:02d}"
            )
            suffix += 1
        self.output_dir.rename(archive_dir)
        return archive_dir

    def build_output(self, image, points, map_points):
        H, errors, rmse = compute_homography(points, map_points)
        output = {
            "image_size": {
                "width": int(image.shape[1]),
                "height": int(image.shape[0]),
            },
            "input_unit": INPUT_UNIT,
            "coordinate_unit": "m",
            "image_points": [
                {"id": f"M{i}", "u": float(u), "v": float(v)}
                for i, (u, v) in enumerate(points, start=1)
            ],
            "map_points": [
                {"id": f"M{i}", "x": float(x), "y": float(y)}
                for i, (x, y) in enumerate(map_points, start=1)
            ],
            "homography": H.tolist(),
            "reprojection_error_m": {
                f"M{i}": float(error)
                for i, error in enumerate(errors, start=1)
            },
            "rmse_m": rmse,
        }
        return output, errors, rmse

    def save(self, reference_image, preview, output):
        self.output_dir.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(
            prefix=f".{self.output_dir.name}_", dir=self.output_dir.parent
        ) as temporary_dir:
            staging_dir = Path(temporary_dir)
            if not cv2.imwrite(
                str(staging_dir / REFERENCE_NAME), reference_image
            ):
                raise RuntimeError("기준 이미지 저장에 실패했습니다.")
            if not cv2.imwrite(str(staging_dir / POINTS_NAME), preview):
                raise RuntimeError("포인트 이미지 저장에 실패했습니다.")
            with open(
                staging_dir / CALIBRATION_NAME, "w", encoding="utf-8"
            ) as file:
                json.dump(output, file, indent=2, ensure_ascii=False)
            archived_dir = self.archive()
            staging_dir.rename(self.output_dir)
        return archived_dir


class CalibrationEditor:
    """라이브/불러오기 공용 마우스 대응점 편집기."""

    def __init__(self, root, points=None, map_points=None):
        self.root = root
        self.points = list(points or [])
        self.map_points = list(map_points or [])
        self.pending_click = None
        self.image_size = None
        self.selected_index = None

    def on_mouse(self, event, x, y, _flags, _param):
        if self.image_size is not None:
            width, height = self.image_size
            x = min(max(x, 1), width - 2)
            y = min(max(y, 1), height - 2)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pending_click = ("upsert", x, y)
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.pending_click = ("delete", x, y)

    def nearest_index(self, x, y):
        if not self.points:
            return None
        distances = [np.hypot(u - x, v - y) for u, v in self.points]
        index = int(np.argmin(distances))
        return index if distances[index] <= MARKER_PICK_RADIUS_PX else None

    def prompt_coordinate(self, point_id, initial=None):
        """GUI에서 mm 좌표를 받아 내부 단위 m로 변환한다."""
        initial_value = ""
        if initial is not None:
            initial_value = (
                f"{initial[0] / MM_TO_M:g} {initial[1] / MM_TO_M:g}"
            )

        while True:
            raw = simpledialog.askstring(
                f"{point_id} 실좌표",
                "X Y [mm]를 입력하세요.\n예: 125.0 340.0",
                initialvalue=initial_value,
                parent=self.root,
            )
            if raw is None:
                return None
            try:
                parts = raw.replace(",", " ").split()
                if len(parts) != 2:
                    raise ValueError
                return float(parts[0]) * MM_TO_M, float(parts[1]) * MM_TO_M
            except ValueError:
                messagebox.showerror(
                    "입력 오류", "X Y 숫자 두 개를 입력하세요.", parent=self.root
                )

    def handle_pending_click(self):
        if self.pending_click is None:
            return

        action, x, y = self.pending_click
        self.pending_click = None
        index = self.nearest_index(x, y)

        if action == "delete":
            if index is not None:
                del self.points[index]
                del self.map_points[index]
                self.selected_index = None
            return

        if index is None:
            coordinate = self.prompt_coordinate(f"M{len(self.points) + 1}")
            if coordinate is not None:
                self.points.append((float(x), float(y)))
                self.map_points.append(coordinate)
                self.selected_index = len(self.points) - 1
        else:
            self.selected_index = index
            coordinate = self.prompt_coordinate(
                f"M{index + 1}", self.map_points[index]
            )
            if coordinate is not None:
                self.map_points[index] = coordinate

    def move_selected(self, dx, dy):
        if self.selected_index is None or self.image_size is None:
            return
        width, height = self.image_size
        u, v = self.points[self.selected_index]
        self.points[self.selected_index] = (
            float(min(max(u + dx, 1), width - 2)),
            float(min(max(v + dy, 1), height - 2)),
        )

    def draw(self, image, live):
        self.image_size = (image.shape[1], image.shape[0])
        marked_image = MarkerRenderer.render_markers(image, self.points)
        display = MarkerRenderer.draw_annotations(
            marked_image, self.points, self.map_points
        )
        mode_text = "LIVE" if live else "LOAD"
        instructions = (
            f"{mode_text} | Left: add/edit  Right: delete  Enter/S: save  Esc/Q: cancel"
        )
        cv2.rectangle(display, (0, 0), (display.shape[1] - 1, 27), (0, 0, 0), -1)
        cv2.putText(
            display, instructions, (8, 19), cv2.FONT_HERSHEY_SIMPLEX,
            0.45, (255, 255, 255), 1, cv2.LINE_AA
        )
        return display


class MarkerMovePanel:
    """선택한 마커를 1 px 단위로 이동하는 GUI 패널."""

    def __init__(self, root, editor):
        self.editor = editor
        self.window = tk.Toplevel(root)
        self.window.title("마커 미세 이동")
        self.window.resizable(False, False)
        self.window.attributes("-topmost", True)
        self.window.protocol("WM_DELETE_WINDOW", self.window.withdraw)
        self.window.deiconify()
        self.window.lift()

        self.selected_text = tk.StringVar(value="선택된 마커 없음")
        tk.Label(
            self.window, textvariable=self.selected_text, padx=14, pady=8
        ).grid(row=0, column=0, columnspan=3)
        tk.Button(
            self.window, text="▲", width=5,
            command=lambda: editor.move_selected(0, -1)
        ).grid(row=1, column=1, padx=3, pady=3)
        tk.Button(
            self.window, text="◀", width=5,
            command=lambda: editor.move_selected(-1, 0)
        ).grid(row=2, column=0, padx=3, pady=3)
        tk.Label(self.window, text="1 px", width=5).grid(row=2, column=1)
        tk.Button(
            self.window, text="▶", width=5,
            command=lambda: editor.move_selected(1, 0)
        ).grid(row=2, column=2, padx=3, pady=3)
        tk.Button(
            self.window, text="▼", width=5,
            command=lambda: editor.move_selected(0, 1)
        ).grid(row=3, column=1, padx=3, pady=(3, 10))

    def refresh(self):
        index = self.editor.selected_index
        if index is None:
            self.selected_text.set("선택된 마커 없음")
            return
        u, v = self.editor.points[index]
        self.selected_text.set(f"M{index + 1}  pixel=({u:.0f}, {v:.0f})")

    def destroy(self):
        self.window.destroy()


def choose_mode(root):
    """탭에서 불러오기/라이브 시작 옵션을 선택한다."""
    root.title("WebCam2 캘리브레이션")
    root.resizable(False, False)
    root.deiconify()
    root.lift()

    camera_indices = sorted(
        int(path.name.removeprefix("video"))
        for path in Path("/dev").glob("video[0-9]*")
        if path.name.removeprefix("video").isdigit()
    )
    if not camera_indices:
        camera_indices = [0]
    selected_camera = tk.StringVar(value=str(camera_indices[0]))
    selected_image = tk.StringVar(value=str(OUTPUT_DIR / REFERENCE_NAME))
    use_json = tk.BooleanVar(value=True)
    selected_json = tk.StringVar(value=str(OUTPUT_DIR / CALIBRATION_NAME))
    result = {"mode": None, "camera": None, "image": None, "json": None}
    selection_done = tk.BooleanVar(value=False)

    notebook = ttk.Notebook(root)
    notebook.pack(fill="both", expand=True, padx=18, pady=(18, 6))
    load_tab = tk.Frame(notebook, padx=18, pady=16)
    live_tab = tk.Frame(notebook, padx=18, pady=16)
    notebook.add(load_tab, text="불러오기")
    notebook.add(live_tab, text="라이브")

    def browse_file(target, title, initial_dir, filetypes):
        image_path = filedialog.askopenfilename(
            title=title,
            initialdir=str(initial_dir),
            filetypes=filetypes,
            parent=root,
        )
        if image_path:
            target.set(image_path)

    tk.Label(load_tab, text="마커 이미지 (필수)").pack(anchor="w")
    image_frame = tk.Frame(load_tab)
    image_frame.pack(fill="x", pady=(4, 14))
    tk.Entry(image_frame, textvariable=selected_image, width=52).pack(
        side="left", fill="x", expand=True
    )
    tk.Button(
        image_frame,
        text="찾아보기",
        command=lambda: browse_file(
            selected_image,
            "마커 이미지 선택",
            OUTPUT_DIR.parent,
            (("이미지", "*.png *.jpg *.jpeg *.bmp"), ("모든 파일", "*")),
        ),
        width=10,
    ).pack(side="right", padx=(8, 0))

    json_check = tk.Checkbutton(
        load_tab,
        text="기존 실좌표 JSON 불러오기 (선택)",
        variable=use_json,
        anchor="w",
    )
    json_check.pack(fill="x")
    json_frame = tk.Frame(load_tab)
    json_frame.pack(fill="x", pady=(4, 0))
    json_entry = tk.Entry(json_frame, textvariable=selected_json, width=52)
    json_entry.pack(side="left", fill="x", expand=True)
    json_button = tk.Button(
        json_frame,
        text="찾아보기",
        command=lambda: browse_file(
            selected_json,
            "캘리브레이션 JSON 선택",
            OUTPUT_DIR.parent,
            (("JSON", "*.json"), ("모든 파일", "*")),
        ),
        width=10,
    )
    json_button.pack(side="right", padx=(8, 0))

    def update_json_state(*_args):
        state = "normal" if use_json.get() else "disabled"
        json_entry.configure(state=state)
        json_button.configure(state=state)

    use_json.trace_add("write", update_json_state)

    camera_frame = tk.Frame(live_tab)
    camera_frame.pack(fill="x")
    tk.Label(camera_frame, text="카메라 인덱스").pack(side="left")
    camera_combo = ttk.Combobox(
        camera_frame,
        textvariable=selected_camera,
        values=[str(index) for index in camera_indices],
        state="readonly",
        width=8,
    )
    camera_combo.pack(side="right")

    def confirm():
        if notebook.index(notebook.select()) == 1:
            result["mode"] = "live"
            result["camera"] = int(selected_camera.get())
        else:
            result["mode"] = "load"
            result["image"] = selected_image.get().strip()
            if not result["image"] or not Path(result["image"]).is_file():
                messagebox.showwarning(
                    "이미지 필요", "사용할 이미지 파일을 선택하세요.", parent=root
                )
                result["mode"] = None
                return
            if use_json.get():
                result["json"] = selected_json.get().strip()
                if not result["json"] or not Path(result["json"]).is_file():
                    messagebox.showwarning(
                        "JSON 필요",
                        "JSON 불러오기를 해제하거나 JSON 파일을 선택하세요.",
                        parent=root,
                    )
                    result["mode"] = None
                    return
        selection_done.set(True)

    def cancel():
        selection_done.set(True)

    buttons = tk.Frame(root, padx=18, pady=12)
    buttons.pack(fill="x")
    tk.Button(buttons, text="취소", command=cancel, width=10).pack(side="right")
    tk.Button(buttons, text="시작", command=confirm, width=10).pack(
        side="right", padx=(0, 8)
    )
    root.protocol("WM_DELETE_WINDOW", cancel)
    root.bind("<Return>", lambda _event: confirm())
    root.bind("<Escape>", lambda _event: cancel())
    root.update_idletasks()
    x = root.winfo_screenwidth() // 2 - root.winfo_width() // 2
    y = root.winfo_screenheight() // 2 - root.winfo_height() // 2
    root.geometry(f"+{x}+{y}")
    root.focus_force()
    root.grab_set()
    root.wait_variable(selection_done)
    root.grab_release()
    root.withdraw()
    return result["mode"], result["camera"], result["image"], result["json"]


def parse_args():
    parser = argparse.ArgumentParser(
        description="WebCam2 마우스 마커 캘리브레이션 GUI"
    )
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    return parser.parse_args()


def run_editor(root, mode, camera_index, image_path, json_path, width, height):
    capture = None
    editor = CalibrationEditor(root)
    store = CalibrationStore(OUTPUT_DIR)

    if mode == "load":
        base_image, points, map_points = store.load(image_path, json_path)
        editor.points = points
        editor.map_points = map_points
    else:
        capture = cv2.VideoCapture(camera_index)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not capture.isOpened():
            raise RuntimeError(f"카메라를 열 수 없습니다: index={camera_index}")
        ok, base_image = capture.read()
        if not ok:
            raise RuntimeError("카메라의 첫 프레임을 읽지 못했습니다.")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW_NAME, editor.on_mouse)
    move_panel = MarkerMovePanel(root, editor)
    saved = False

    try:
        while True:
            if capture is not None:
                ok, frame = capture.read()
                if ok:
                    base_image = frame

            editor.handle_pending_click()
            move_panel.refresh()
            root.update_idletasks()
            root.update()
            display = editor.draw(base_image, live=capture is not None)
            cv2.imshow(WINDOW_NAME, display)
            key = cv2.waitKey(20) & 0xFF

            if key in (13, 10, ord("s")):
                missing_coordinates = sum(
                    point is None for point in editor.map_points
                )
                if len(editor.points) < 4 or missing_coordinates:
                    messagebox.showwarning(
                        "캘리브레이션 미완성",
                        "Homography 계산에는 마커가 최소 4개 필요하고 "
                        "모든 마커에 실좌표가 있어야 합니다.\n"
                        f"현재 마커: {len(editor.points)}개, "
                        f"좌표 미입력: {missing_coordinates}개",
                        parent=root,
                    )
                    continue
                saved = True
                break
            if key in (27, ord("q")):
                break
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        move_panel.destroy()
        if capture is not None:
            capture.release()
        cv2.destroyAllWindows()

    if not saved:
        return None

    reference_image = MarkerRenderer.render_markers(base_image, editor.points)
    preview = MarkerRenderer.draw_annotations(
        reference_image, editor.points, editor.map_points
    )
    output, errors, rmse = store.build_output(
        reference_image, editor.points, editor.map_points
    )
    archived_dir = store.save(reference_image, preview, output)
    return archived_dir, errors, rmse


def main():
    args = parse_args()
    root = tk.Tk()
    root.withdraw()

    try:
        mode, camera_index, image_path, json_path = choose_mode(root)
        if mode is None:
            return
        result = run_editor(
            root, mode, camera_index, image_path, json_path,
            args.width, args.height
        )
        if result is None:
            print("캘리브레이션을 저장하지 않고 종료했습니다.")
            return

        archived_dir, errors, rmse = result
        print("\n=== Reprojection error ===")
        for i, error in enumerate(errors, start=1):
            print(f"M{i:02d}: {error:.4f} m")
        print(f"RMSE: {rmse:.4f} m\n")
        if archived_dir is not None:
            print(f"기존 캘리브레이션 보존: {archived_dir}")
        print(f"기준 이미지: {OUTPUT_DIR / REFERENCE_NAME}")
        print(f"포인트 번호 이미지: {OUTPUT_DIR / POINTS_NAME}")
        print(f"캘리브레이션 결과: {OUTPUT_DIR / CALIBRATION_NAME}")
    except (FileNotFoundError, KeyError, ValueError, RuntimeError) as error:
        messagebox.showerror("캘리브레이션 오류", str(error), parent=root)
        raise
    finally:
        root.destroy()


if __name__ == "__main__":
    main()
