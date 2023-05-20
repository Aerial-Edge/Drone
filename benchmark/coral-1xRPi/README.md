Can be run on any Raspberry Pi with a 64-bit OS and an attached Coral USB Accelerator.

To benchmark the "EfficientDet Lite 0" model (default) inference, run the command:

$ docker compose run --rm benchmark


To benchmark the "EfficientDet Lite 1" mode inference, run the command:

$ docker compose run --rm benchmark python3 ./app/detect_edl1.py


...while in the root of this directory.