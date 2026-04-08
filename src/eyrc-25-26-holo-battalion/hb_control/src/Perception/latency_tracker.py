import cv2
import time
import matplotlib.pyplot as plt
from collections import deque

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

# Store last 100 latency values
latencies = deque(maxlen=100)

plt.ion()
fig, ax = plt.subplots()

while True:
    t0 = time.time()

    ret, frame = cap.read()
    t1 = time.time()

    # ---- YOUR VISION PROCESSING HERE ----
    # Example dummy processing
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    t2 = time.time()

    capture_delay = (t1 - t0) * 1000
    processing_delay = (t2 - t1) * 1000
    total_latency = (t2 - t0) * 1000

    latencies.append(total_latency)

    # Display latency on frame
    cv2.putText(frame, f"Latency: {total_latency:.1f} ms",
                (20,40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    cv2.imshow("Camera", frame)

    # Live plot update
    ax.clear()
    ax.plot(latencies)
    ax.set_title("Vision Latency (ms)")
    ax.set_ylabel("ms")
    ax.set_xlabel("Frame")
    plt.pause(0.001)

    print(f"Capture: {capture_delay:.1f} ms | Proc: {processing_delay:.1f} ms | Total: {total_latency:.1f} ms")

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
