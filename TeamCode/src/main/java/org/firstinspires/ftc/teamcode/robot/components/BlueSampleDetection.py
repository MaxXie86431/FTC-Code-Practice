import cv2
from inference_sdk import InferenceHTTPClient, InferenceConfiguration

CLIENT = InferenceHTTPClient(
    api_url="https://detect.roboflow.com",  # or "https://serverless.roboflow.com"
    api_key="dgzY8pSPBRBJD1zILBDd"
)

# Optional: make detection threshold more sensitive so you actually see output
CLIENT.configure(InferenceConfiguration(confidence_threshold=0.15))

model_id = "sampledetector/10"  # Use your exact <project-name>/<version-number>

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    raise RuntimeError("Could not open webcam (device 0)")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame capture failed; exiting loop.")
        break

    # cv2 returns BGR but InferenceHTTPClient works fine with numpy arrays (i.e. cv2 itself)
    result = CLIENT.infer(frame, model_id=model_id)
    # With numpy input, returns a dict with “predictions” list—possibly empty if no detections above threshold
    print("Frame points:", len(result.get("predictions", [])))

    for pred in result["predictions"]:
        x, y, w, h = pred["x"], pred["y"], pred["width"], pred["height"]
        cls, conf = pred["class"], pred["confidence"]
        x1, y1 = int(x - w / 2), int(y - h / 2)
        x2, y2 = int(x + w / 2), int(y + h / 2)
        # draw bounding box + label
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"{cls} {conf:.2f}",
                    (x1, max(0, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Detections", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
