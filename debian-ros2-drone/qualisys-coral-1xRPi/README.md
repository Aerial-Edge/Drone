To start, run:

$ docker compose up

To run in detached mode, run:

$ docker compose up -d


To destroy containers, run:

$ docker compose down


...while in the root of this directory.


DISCLAIMER: If Coral USB Accelerator has experienced a power off-on cycle it will throw an error on first inference due to its flashing process:
"ValueError: Failed to load delegate from libedgetpu.so.1.0"

Run it again and the Coral USB Accelerator should function as expected.