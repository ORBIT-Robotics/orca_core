from orca_core import OrcaHand
import time  
import argparse  


def main(): 
    parser = argparse.ArgumentParser(description="Test the ORCA Hand.")  # Added parser
    parser.add_argument('model_path', type=str, nargs='?', default=None, help='Path to the hand model directory')
    args = parser.parse_args()
    hand = OrcaHand(args.model_path)

    status = hand.connect()
    print(status)
    if not status[0]:
        print("Failed to connect to the hand.")
        exit(1)

    hand.enable_torque()

    joint_dict = {
        # Middle finger: middle extended, others curled inward
        "thumb_mcp": 30,
        "thumb_abd": 22,
        "thumb_dip": 40,
        "thumb_pip": 50,
        "index_abd": 0,
        "index_mcp": 75,
        "index_pip": 85,
        "middle_abd": -5,
        "middle_mcp": -15,
        "middle_pip": -12,
        "ring_mcp": 85,
        "ring_pip": 95,
        "pinky_mcp": 90,
        "pinky_pip": 100,
    }

    hand.set_joint_pos(joint_dict, num_steps=25, step_size=0.001)

    time.sleep(2)
    hand.disable_torque()

    hand.disconnect()

if __name__ == "__main__":  # Added main execution block
    main()
