# This contains the full state of the air vehicle. More fields to be added later
float32 pn        # Inertial North position
float32 pe        # Inertial East position
float32 pd        # Inertial Down position
float32 u         # Body frame velocity along i^(body)
float32 v         # Body frame velocity along j^(body)
float32 w         # Body frame velocity along k^(body)
float32 phi       # Roll, angle rotated around i^(vehicle-2)
float32 theta     # Pitch, angle rotated around j^(vehicle-1)
float32 psi       # Yaw, angle rotated around k^(vehicle)
float32 p         # Roll rate around i^(body)
float32 q         # Pitch rate around j^(body)
float32 r         # Yaw rate around k^(body)
