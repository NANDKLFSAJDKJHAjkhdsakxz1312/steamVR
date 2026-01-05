#include "vr_devices.h"
#include <iostream>

int main(int argc, char **argv) {
	std::mutex data_mutex;
	std::condition_variable data_cv;
	vr_controller_data shared_data;

	double publish_frequency = VRInputConfig::MAX_PUBLISH_FREQUENCY;

	try {
		if (publish_frequency <= VRInputConfig::MIN_PUBLISH_FREQUENCY || publish_frequency > VRInputConfig::MAX_PUBLISH_FREQUENCY) {
			std::cerr << "Invalid frequency. Using default " << VRInputConfig::DEFAULT_PUBLISH_FREQUENCY << "Hz." << std::endl;
			publish_frequency = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY;
		}
	}
	catch (const std::exception& e) {
		std::cerr << "Invalid frequency argument. Using default " << VRInputConfig::DEFAULT_PUBLISH_FREQUENCY << "Hz." << std::endl;
		publish_frequency = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY;
	}

	vive_input viveInput(data_mutex, data_cv, shared_data, publish_frequency);
	viveInput.runVR();

	return 0;
}
