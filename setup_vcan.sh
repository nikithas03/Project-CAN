sudo modprobe vcan
# Create a virtual CAN interface
sudo ip link add dev vcan0 type vcan
# Bring up the interface
sudo ip link set up vcan0

echo "vcan0 is now set up and ready for CAN communication."
