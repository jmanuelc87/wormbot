#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

#include "windows.h"
#include "WinError.h"
#include "xinput.h"

#define MAX_CONTROLLERS 1
#define INPUT_DEADZONE (0.27F * FLOAT(0X7FFF))

typedef struct CONTROLLER_STATE {
	XINPUT_STATE state;
	bool bConnected;
} ControllerState;

ControllerState g_Controllers[MAX_CONTROLLERS];
bool g_bDeadZoneOn = true;
WCHAR g_szMessage[MAX_CONTROLLERS][1024] = { 0 };


HRESULT UpdateControllersState();
void RenderFrame();
void updateTopic(ros::Publisher pub);
float normalize(SHORT value);
float normalize(BYTE value);
void debug();

int main(int argc, char * argv[])
{
	ros::init(argc, argv, "joy_control_node");

	ros::NodeHandle nh;

	std::string topic = nh.getNamespace() + "/joy";

	ros::Publisher pub = nh.advertise<sensor_msgs::Joy>(topic, 1);

	ros::Rate loop(50);

	while (ros::ok()) {

		UpdateControllersState();
		RenderFrame();
		updateTopic(pub);
		//debug();

		loop.sleep();
	}

	return 0;
}

HRESULT UpdateControllersState() {
	DWORD dwResult;
	for (int i = 0; i < MAX_CONTROLLERS; i++) {
		dwResult = XInputGetState(i, &g_Controllers[i].state);

		if (dwResult == ERROR_SUCCESS) {
			g_Controllers[i].bConnected = true;
		}
		else {
			g_Controllers[i].bConnected = false;
		}
	}
	return S_OK;
}

void RenderFrame() {
	WCHAR sz[MAX_CONTROLLERS][1024];
	for (DWORD i = 0; i < MAX_CONTROLLERS; i++) {
		if (g_Controllers[i].bConnected) {
			WORD wButtons = g_Controllers[i].state.Gamepad.wButtons;

			if (g_bDeadZoneOn) {

				if (g_Controllers[i].state.Gamepad.sThumbLX < INPUT_DEADZONE &&
					g_Controllers[i].state.Gamepad.sThumbLX > -INPUT_DEADZONE)
				{
					g_Controllers[i].state.Gamepad.sThumbLX = 0;
				}

				if (g_Controllers[i].state.Gamepad.sThumbLY < INPUT_DEADZONE &&
					g_Controllers[i].state.Gamepad.sThumbLY > -INPUT_DEADZONE)
				{
					g_Controllers[i].state.Gamepad.sThumbLY = 0;
				}

				if (g_Controllers[i].state.Gamepad.sThumbRX < INPUT_DEADZONE &&
					g_Controllers[i].state.Gamepad.sThumbRX > -INPUT_DEADZONE)
				{
					g_Controllers[i].state.Gamepad.sThumbRX = 0;
				}

				if (g_Controllers[i].state.Gamepad.sThumbRY < INPUT_DEADZONE &&
					g_Controllers[i].state.Gamepad.sThumbRY > -INPUT_DEADZONE)
				{
					g_Controllers[i].state.Gamepad.sThumbRY = 0;
				}
			}

			swprintf_s(sz[i], 1024,
				L"Controller %u: Connected\n"
				L"  Buttons: %s%s%s%s%s%s%s%s%s%s%s%s%s%s\n"
				L"  Left Trigger: %u\n"
				L"  Right Trigger: %u\n"
				L"  Left Thumbstick: %d/%d\n"
				L"  Right Thumbstick: %d/%d", i,
				(wButtons & XINPUT_GAMEPAD_DPAD_UP) ? L"DPAD_UP " : L"",
				(wButtons & XINPUT_GAMEPAD_DPAD_DOWN) ? L"DPAD_DOWN " : L"",
				(wButtons & XINPUT_GAMEPAD_DPAD_LEFT) ? L"DPAD_LEFT " : L"",
				(wButtons & XINPUT_GAMEPAD_DPAD_RIGHT) ? L"DPAD_RIGHT " : L"",
				(wButtons & XINPUT_GAMEPAD_START) ? L"START " : L"",
				(wButtons & XINPUT_GAMEPAD_BACK) ? L"BACK " : L"",
				(wButtons & XINPUT_GAMEPAD_LEFT_THUMB) ? L"LEFT_THUMB " : L"",
				(wButtons & XINPUT_GAMEPAD_RIGHT_THUMB) ? L"RIGHT_THUMB " : L"",
				(wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER) ? L"LEFT_SHOULDER " : L"",
				(wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER) ? L"RIGHT_SHOULDER " : L"",
				(wButtons & XINPUT_GAMEPAD_A) ? L"A " : L"",
				(wButtons & XINPUT_GAMEPAD_B) ? L"B " : L"",
				(wButtons & XINPUT_GAMEPAD_X) ? L"X " : L"",
				(wButtons & XINPUT_GAMEPAD_Y) ? L"Y " : L"",
				g_Controllers[i].state.Gamepad.bLeftTrigger,
				g_Controllers[i].state.Gamepad.bRightTrigger,
				g_Controllers[i].state.Gamepad.sThumbLX,
				g_Controllers[i].state.Gamepad.sThumbLY,
				g_Controllers[i].state.Gamepad.sThumbRX,
				g_Controllers[i].state.Gamepad.sThumbRY);
		}
		else {
			swprintf_s(sz[i], 1024, L"Controller %u: Not connected", i);
		}

		if (wcscmp(sz[i], g_szMessage[i]) != 0)
		{
			wcscpy_s(g_szMessage[i], 1024, sz[i]);
		}
	}
}

void updateTopic(ros::Publisher pub) {
	for (DWORD i = 0; i < MAX_CONTROLLERS; i++) {
		if (g_Controllers[i].bConnected) {
			sensor_msgs::Joy joy;

			joy.axes.push_back(normalize(g_Controllers[i].state.Gamepad.sThumbLX));
			joy.axes.push_back(normalize(g_Controllers[i].state.Gamepad.sThumbLY));
			joy.axes.push_back(normalize(g_Controllers[i].state.Gamepad.sThumbRX));
			joy.axes.push_back(normalize(g_Controllers[i].state.Gamepad.sThumbRY));
			joy.axes.push_back(normalize(g_Controllers[i].state.Gamepad.bLeftTrigger));
			joy.axes.push_back(normalize(g_Controllers[i].state.Gamepad.bRightTrigger));

			WORD wButtons = g_Controllers[i].state.Gamepad.wButtons;

			joy.buttons.push_back(wButtons & XINPUT_GAMEPAD_DPAD_UP ? 1 : 0);
			joy.buttons.push_back(wButtons & XINPUT_GAMEPAD_DPAD_DOWN ? 1 : 0);
			joy.buttons.push_back(wButtons & XINPUT_GAMEPAD_DPAD_LEFT ? 1 : 0);
			joy.buttons.push_back(wButtons & XINPUT_GAMEPAD_DPAD_RIGHT ? 1 : 0);

			joy.buttons.push_back(wButtons & XINPUT_GAMEPAD_A ? 1 : 0);
			joy.buttons.push_back(wButtons & XINPUT_GAMEPAD_B ? 1 : 0);
			joy.buttons.push_back(wButtons & XINPUT_GAMEPAD_X ? 1 : 0);
			joy.buttons.push_back(wButtons & XINPUT_GAMEPAD_Y ? 1 : 0);

			joy.buttons.push_back(wButtons & XINPUT_GAMEPAD_START ? 1 : 0);
			joy.buttons.push_back(wButtons & XINPUT_GAMEPAD_BACK ? 1 : 0);

			pub.publish(joy);
		}
	}
}

float normalize(SHORT value) {
	return (float)value / 32767.0f;
}

float normalize(BYTE value) {
	return (float)value / 255.0f;
}

void debug() {
	char output[1024];
	for (int i = 0; i < MAX_CONTROLLERS; i++) {
		sprintf_s(output, "%ws", g_szMessage[i]);
		ROS_INFO_THROTTLE_NAMED(3, "CONTROLLERS", output);
	}
}
