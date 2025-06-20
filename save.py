#Add a State Machine to Handle Vehicle Behavior
#This allows the vehicle to change behavior based on signs and pedestrian detections.
class VehicleState:
    NORMAL = "normal"
    SLOWING_DOWN = "slowing_down"
    WAITING_FOR_PEDESTRIAN = "waiting_for_pedestrian"
    STOPPED = "stopped"
    RESUMING = "resuming"

#Create a Log Structure
#At the start of your main script:
import json
import time

log_data = {
    "experiment_id": "blue_red_sign_behavior_test_01",
    "events": []
}
#And a helper function to log events:
def log_event(event_type, data):
    log_data["events"].append({
        "timestamp": time.time(),
        "event": event_type,
        **data
    })


#Update Your Loop with FSM + Detection Logic
#Add state + timers at the start:
state = VehicleState.NORMAL
stop_start_time = None

#Inside your main control loop:
# Pseudocode for detection outputs (replace with actual model results)
detections = yolo_model(frame)  # you already have this
red_sign_detected = detect_label(detections, "red_ped_sign")
blue_sign_detected = detect_label(detections, "blue_ped_sign")
pedestrian_detected = detect_label(detections, "pedestrian")


if state == VehicleState.NORMAL:
    if red_sign_detected:
        state = VehicleState.SLOWING_DOWN
        log_event("red_sign_detected", {...})

elif state == VehicleState.SLOWING_DOWN:
    # Apply slower speed (15–20 km/h)
    # Once blue sign detected → check for pedestrian
    if blue_sign_detected:
        if pedestrian_detected:
            state = VehicleState.STOPPED
            stop_start_time = time.time()
            log_event("blue_sign_and_pedestrian", {...})
        else:
            state = VehicleState.WAITING_FOR_PEDESTRIAN
            log_event("blue_sign_no_pedestrian", {...})

elif state == VehicleState.WAITING_FOR_PEDESTRIAN:
    if pedestrian_detected:
        state = VehicleState.STOPPED
        stop_start_time = time.time()
        log_event("late_pedestrian_detected", {...})
    else:
        # Resume slowly after a few seconds
        state = VehicleState.RESUMING
        log_event("no_pedestrian_resume", {...})

elif state == VehicleState.STOPPED:
    if time.time() - stop_start_time >= 20:
        state = VehicleState.RESUMING
        log_event("resume_after_stop", {
            "stopped_duration": time.time() - stop_start_time
        })

elif state == VehicleState.RESUMING:
    # Resume normal speed and reset
    state = VehicleState.NORMAL

#Save the Log File at the End
#At the end of your simulation:
with open("ped_sign_behavior_log.json", "w") as f:
    json.dump(log_data, f, indent=2)

#Optional Enhancements
#Log vehicle speed, acceleration at every event.
#Capture image frame where detection occurred:
image.save_to_disk(f"frames/{timestamp}.jpg")
