#define NOMINMAX

#include <windows.h>			
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <btBulletDynamicsCommon.h>	
#include <BulletCollision\CollisionDispatch\btGhostObject.h>
#include <BulletDynamics\Character\btKinematicCharacterController.h>
#include "GL.h"
#include "Extension.h"

#include "MeshObject\MeshSphere.h"
#include "MeshObject\MeshQuad.h"
#include "MeshObject\MeshCube.h"
#include "MeshObject\MeshTorus.h"
#include "MeshObject\MeshSpiral.h"
#include "Camera.h"
#include "Skybox.h"
#include "DynamicCharacterController.h"

int height = 960;
int width = 1280;

POINT g_OldCursorPos;
bool g_enableVerticalSync;
bool g_enableWireframe;

enum collisiontypes {
	COL_NOTHING = 0,
	COL_SPHERE1 = 1,
	COL_SPHERE2 = 2,
	COL_GHOST = 4,
	COL_BOX = 8,

	COL_FORCE_8BIT = 0xFF
};

enum DIRECTION {
	DIR_FORWARD = 1,
	DIR_BACKWARD = 2,
	DIR_LEFT = 4,
	DIR_RIGHT = 8,
	DIR_UP = 16,
	DIR_DOWN = 32,
	DIR_FORWARD2 = 64,
	DIR_BACKWARD2 = 128,
	DIR_LEFT2 = 256,
	DIR_RIGHT2 = 512,

	DIR_FORCE_32BIT = 0x7FFFFFFF
};

struct ColPoint {
	btVector3 position = btVector3(0.0f, 0.0f, 0.0f);
	btTransform transform;
	bool active = false;
	bool upward = false;

	float angle = 0.0;
	float rad = 0.0;
};

SkyBox* skyBox;
Camera camera;
MeshSphere *sphere;
MeshQuad *quad;
MeshCube *cube, *cube2;
bool UPDATEFRAME = false;
double delta_Time;

btDynamicsWorld* world;
btDispatcher* dispatcher;
btCollisionConfiguration* collisionConfig;
btBroadphaseInterface* broadphase;
btConstraintSolver* solver;

btRigidBody* m_groundBody;
void kinematicPreTickCallback(btDynamicsWorld* world, btScalar deltaTime);

DynamicCharacterController *dynamicCharacterController;
btKinematicCharacterController* charCon;
btPairCachingGhostObject* ghostObject;
bool upward = true;
float velY = 0.0;
float  angVelY = 0.1;

ColPoint colPoint;

//prototype funktions
LRESULT CALLBACK winProc(HWND hWnd, UINT message, WPARAM wParma, LPARAM lParam);
void setCursortoMiddle(HWND hwnd);
void enableWireframe(bool enableWireframe);
void enableVerticalSync(bool enableVerticalSync);

void initApp(HWND hWnd);
void processInput(HWND hWnd);
void updateDyn(HWND hWnd, double elapsedTimeSec);
void updateKin(HWND hWnd, double elapsedTimeSec);
// the main windows entry point
int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nShowCmd) {

	AllocConsole();
	AttachConsole(GetCurrentProcessId());
	freopen("CON", "w", stdout);
	SetConsoleTitle("Debug console");

	MoveWindow(GetConsoleWindow(), 1300, 0, 550, 300, true);
	std::cout << "w, a, s, d, q, e, mouse : move camera" << std::endl;
	std::cout << "arrow keys              : move player" << std::endl;
	std::cout << "control right           : jump" << std::endl;
	std::cout << "mouse left              : shoot sphere" << std::endl;
	std::cout << "space                   : release capture" << std::endl;
	std::cout << "v                       : toggle vsync" << std::endl;
	std::cout << "z                       : toggle wireframe" << std::endl;

	WNDCLASSEX		windowClass;		// window class
	HWND			hwnd;				// window handle
	MSG				msg;				// message
	HDC				hdc;				// device context handle

										// fill out the window class structure
	windowClass.cbSize = sizeof(WNDCLASSEX);
	windowClass.style = CS_HREDRAW | CS_VREDRAW;
	windowClass.lpfnWndProc = winProc;
	windowClass.cbClsExtra = 0;
	windowClass.cbWndExtra = 0;
	windowClass.hInstance = hInstance;
	windowClass.hIcon = LoadIcon(NULL, IDI_APPLICATION);		// default icon
	windowClass.hCursor = LoadCursor(NULL, IDC_ARROW);			// default arrow
	windowClass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);	// white background
	windowClass.lpszMenuName = NULL;									// no menu
	windowClass.lpszClassName = "WINDOWCLASS";
	windowClass.hIconSm = LoadIcon(NULL, IDI_WINLOGO);			// windows logo small icon

																// register the windows class
	if (!RegisterClassEx(&windowClass))
		return 0;

	// class registered, so now create our window
	hwnd = CreateWindowEx(NULL,									// extended style
		"WINDOWCLASS",						// class name
		"Physics",					// app name
		WS_OVERLAPPEDWINDOW,
		0, 0,									// x,y coordinate
		width,
		height,									// width, height
		NULL,									// handle to parent
		NULL,									// handle to menu
		hInstance,							// application instance
		NULL);								// no extra params

											// check if window creation failed (hwnd would equal NULL)
	if (!hwnd)
		return 0;

	ShowWindow(hwnd, SW_SHOW);			// display the window
	UpdateWindow(hwnd);					// update the window

	initApp(hwnd);

	std::chrono::steady_clock::time_point start = std::chrono::high_resolution_clock::now();
	std::chrono::steady_clock::time_point end;
	std::chrono::duration<double> deltaTime;

	// main message loop
	while (true) {

			// Did we recieve a message, or are we idling ?
			if (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {

				// test if this is a quit
				if (msg.message == WM_QUIT) break;
				// translate and dispatch message
				TranslateMessage(&msg);
				DispatchMessage(&msg);

			}else {

			end = start;
			start = std::chrono::high_resolution_clock::now();
			deltaTime = start - end;
			delta_Time = deltaTime.count();

			glClearColor(1.0, 1.0, 1.0, 1.0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			dynamicCharacterController->preStep();
			world->stepSimulation(1.0 / 60.0);
			dynamicCharacterController->postStep();

			skyBox->draw(camera);

			{
				btManifoldArray	manifoldArray;
				btBroadphasePairArray& pairArray = ghostObject->getOverlappingPairCache()->getOverlappingPairArray();
				int numPairs = pairArray.size();

				for (int i = 0; i<numPairs; i++){

					manifoldArray.clear();

					const btBroadphasePair& pair = pairArray[i];

					btBroadphasePair* collisionPair = broadphase->getOverlappingPairCache()->findPair(pair.m_pProxy0, pair.m_pProxy1);
					if (!collisionPair)
							continue;

					if (collisionPair->m_algorithm)
					collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);

					for (int j = 0; j<manifoldArray.size(); j++){

						btPersistentManifold* manifold = manifoldArray[j];
						for (int p = 0; p<manifold->getNumContacts(); p++){

							const btManifoldPoint&pt = manifold->getContactPoint(p);

							const btVector3& ptA = pt.getPositionWorldOnA();
							const btVector3& ptB = pt.getPositionWorldOnB();
							const btVector3& normalOnB = pt.m_normalWorldOnB;

							if (!colPoint.active) {
								colPoint.position = ptB + btVector3(0, 5.0, 0);		
								colPoint.active = true;	
							}
						}
					}
				}
			}

			btTransform t;
			m_groundBody->getMotionState()->getWorldTransform(t);
			float mat[16];
			t.getOpenGLMatrix(mat);
			quad->setTransformation(Matrix4f(mat[0], mat[4], mat[8], mat[12],
				mat[1], mat[5], mat[9], mat[13],
				mat[2], mat[6], mat[10], mat[14],
				mat[3], mat[7], mat[11], mat[15]));
			quad->draw(camera);

			dynamicCharacterController->getRigidBody()->getMotionState()->getWorldTransform(t);
			t.getOpenGLMatrix(mat);
			cube->setTransformation(Matrix4f(mat[0], mat[4], mat[8], mat[12],
				mat[1], mat[5], mat[9], mat[13],
				mat[2], mat[6], mat[10], mat[14],
				mat[3], mat[7], mat[11], mat[15]));
			cube->draw(camera);


			t = charCon->getGhostObject()->getWorldTransform();
			t.getOpenGLMatrix(mat);
			sphere->setTransformation(Matrix4f(mat[0], mat[4], mat[8], mat[12],
				mat[1], mat[5], mat[9], mat[13],
				mat[2], mat[6], mat[10], mat[14],
				mat[3], mat[7], mat[11], mat[15]));
			sphere->draw(camera);

			processInput(hwnd);
			updateDyn(hwnd, delta_Time);
			updateKin(hwnd, delta_Time);
			hdc = GetDC(hwnd);
			SwapBuffers(hdc);
			ReleaseDC(hwnd, hdc);
		}
	} // end while

	delete cube;
	delete quad;
	delete skyBox;

	/*for (int i = 0; i<bodies.size(); i++){
	world->removeCollisionObject(bodies[i]);
	btMotionState* motionState = bodies[i]->getMotionState();
	btCollisionShape* shape = bodies[i]->getCollisionShape();
	delete bodies[i];
	delete shape;
	delete motionState;
	}
	//delete dispatcher;
	delete collisionConfig;
	delete solver;
	//delete broadphase;

	btCollisionShape* shape = ghostObject->getCollisionShape();
	//delete ghostObject;
	//delete shape;
	//delete charCon;
	delete world;*/

	return msg.wParam;
}

// the Windows Procedure event handler
LRESULT CALLBACK winProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam) {

	static HGLRC hRC;					// rendering context
	static HDC hDC;						// device context
	POINT pt;
	RECT rect;

	switch (message) {

	case WM_DESTROY: {

		PostQuitMessage(0);
		return 0;
	}

	case WM_CREATE: {

		GetClientRect(hWnd, &rect);
		g_OldCursorPos.x = rect.right / 2;
		g_OldCursorPos.y = rect.bottom / 2;
		pt.x = rect.right / 2;
		pt.y = rect.bottom / 2;
		//SetCursorPos(pt.x, pt.y);
		// set the cursor to the middle of the window and capture the window via "SendMessage"
		SendMessage(hWnd, WM_LBUTTONDOWN, MK_LBUTTON, MAKELPARAM(pt.x, pt.y));
		return 0;
	}break;

	case WM_LBUTTONDOWN: { // Capture the mouse

		setCursortoMiddle(hWnd);
		SetCapture(hWnd);

		return 0;
	} break;

	case WM_KEYDOWN: {

		switch (wParam) {

		case VK_ESCAPE: {

			PostQuitMessage(0);
			return 0;

		}break;
		case VK_SPACE: {

			ReleaseCapture();
			return 0;

		}break;
		case VK_CONTROL: {
			//just use left control
			if ((lParam & 0x01000000) == 0) {

			}
		}break;
		case 'v': case 'V': {
			enableVerticalSync(!g_enableVerticalSync);
			return 0;

		}break;
		case 'z': case 'Z': {
			enableWireframe(!g_enableWireframe);
		}break;

			return 0;
		}break;

		return 0;
	}break;

	case WM_SIZE: {

		int _height = HIWORD(lParam);		// retrieve width and height
		int _width = LOWORD(lParam);

		if (_height == 0) {					// avoid divide by zero
			_height = 1;
		}

		glViewport(0, 0, _width, _height);
		camera.perspective(45.0f, static_cast<float>(_width) / static_cast<float>(_height), 1.0f, 5000.0f);


		return 0;
	}break;

	default:
		break;
	}
	return (DefWindowProc(hWnd, message, wParam, lParam));
}

void initApp(HWND hWnd) {

	static HGLRC hRC;					// rendering context
	static HDC hDC;						// device context

	hDC = GetDC(hWnd);
	int nPixelFormat;					// our pixel format index

	static PIXELFORMATDESCRIPTOR pfd = {
		sizeof(PIXELFORMATDESCRIPTOR),	// size of structure
		1,								// default version
		PFD_DRAW_TO_WINDOW |			// window drawing support
		PFD_SUPPORT_OPENGL |			// OpenGL support
		PFD_DOUBLEBUFFER,				// double buffering support
		PFD_TYPE_RGBA,					// RGBA color mode
		32,								// 32 bit color mode
		0, 0, 0, 0, 0, 0,				// ignore color bits, non-palettized mode
		8,								// no alpha buffer
		0,								// ignore shift bit
		0,								// no accumulation buffer
		0, 0, 0, 0,						// ignore accumulation bits
		24,								// 24 bit z-buffer size
		8,								// no stencil buffer
		0,								// no auxiliary buffer
		PFD_MAIN_PLANE,					// main drawing plane
		0,								// reserved
		0, 0, 0 };						// layer masks ignored

	nPixelFormat = ChoosePixelFormat(hDC, &pfd);	// choose best matching pixel format
	SetPixelFormat(hDC, nPixelFormat, &pfd);		// set pixel format to device context

													// create rendering context and make it current
	hRC = wglCreateContext(hDC);
	wglMakeCurrent(hDC, hRC);
	ReleaseDC(hWnd, hDC);
	enableVerticalSync(true);

	glEnable(GL_DEPTH_TEST);					// hidden surface removal
	glEnable(GL_CULL_FACE);						// do not calculate inside of poly's

												//setup the camera.
	camera = Camera(Vector3f(0.0f, 20.0f, 220.f), Vector3f(0.0f, 20.0f, 0.0f), Vector3f(0.0f, 1.0f, 0.0f));
	camera.perspective(45.0f, static_cast<float>(width) / static_cast<float>(height), 1.0f, 5000.0f);

	//initialize the skybox
	skyBox = new SkyBox("../skyboxes/sor_sea", 1000, false, false, Vector3f(0.0f, 0.5f, 0.0f));
	//skyBox = new SkyBox("../skyboxes/sea", 1000, false, true, Vector3f(0.0f, 0.5f, 0.0f));
	//skyBox = new SkyBox("../skyboxes/hw_morning", 1000, false, false, Vector3f(0.0f, 0.5f, 0.0f));
	skyBox->setProjectionMatrix(camera.getProjectionMatrix());

	//setup some meshes
	quad = new MeshQuad(400, 400, ".\\res\\floor_color_map.png");
	quad->setPrecision(1, 1);
	quad->buildMesh();

	cube = new MeshCube(10.0, 10.0, 10.0, ".\\res\\marble.png");
	cube->setPrecision(1, 1);
	cube->buildMesh4Q();

	cube2 = new MeshCube(10.0, 10.0, 10.0, ".\\res\\darkchecker.png");
	cube2->setPrecision(1, 1);
	cube2->buildMesh4Q();

	sphere = new MeshSphere(Vector3f(0.0f, 0.0f, 0.0), 5.0f, ".\\res\\earth2048.png");
	sphere->setPrecision(20, 20);
	sphere->buildMesh();

	//pretty much initialize everything logically
	collisionConfig = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfig);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();
	world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
	world->setGravity(btVector3(0, -100, 0));	//gravity on Earth

	//create ground
	btBoxShape* groundShape = new btBoxShape(btVector3(btScalar(200.), btScalar(0.1), btScalar(200.)));
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setRotation(btQuaternion(btVector3(0, 0, 1), (PI * 0.0) / 180.0f));

	btVector3 localInertiaGround(0, 0, 0);
	btDefaultMotionState* motionStateGround = new btDefaultMotionState(groundTransform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(0.0, motionStateGround, groundShape, localInertiaGround);
	m_groundBody = new btRigidBody(cInfo);
	m_groundBody->setUserIndex(-1);
	m_groundBody->forceActivationState(DISABLE_DEACTIVATION);
	m_groundBody->setCollisionFlags(btCollisionObject::CF_KINEMATIC_OBJECT | btCollisionObject::CF_STATIC_OBJECT);
	world->addRigidBody(m_groundBody, COL_BOX, COL_GHOST | COL_BOX);
	world->setInternalTickCallback(kinematicPreTickCallback, m_groundBody, true);

	//create dynamic character
	btBoxShape* charShape = new btBoxShape(btVector3(5., 5., 5.));
	btTransform charTransform;
	charTransform.setIdentity();
	charTransform.setOrigin(btVector3(btScalar(-160.), btScalar(60.), btScalar(0.)));
	btVector3 localInertiaChar(0, 0, 0);
	btDefaultMotionState* motionStateChar = new btDefaultMotionState(charTransform);
	btRigidBody::btRigidBodyConstructionInfo cInfoChar(10.0, motionStateChar, charShape, localInertiaChar);

	dynamicCharacterController = new DynamicCharacterController();
	dynamicCharacterController->create(new btRigidBody(cInfoChar), world, COL_GHOST, COL_BOX | COL_GHOST);
	dynamicCharacterController->setSlopeAngle(30);
	dynamicCharacterController->setDistanceOffset(0.1f);

	//create kinaematic player
	broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

	btTransform startTransform;
	startTransform.setIdentity();

	Vector3f pos = camera.getPosition();
	startTransform.setOrigin(btVector3(btScalar(160.), btScalar(160.), btScalar(0.)));
	btConvexShape* ghostShape = new btSphereShape(5.0);
	//btConvexShape* ghostShape = new btBoxShape(btVector3(5., 5., 5.));
	ghostObject = new btPairCachingGhostObject();
	ghostObject->setCollisionShape(ghostShape);
	ghostObject->setWorldTransform(startTransform);
	ghostObject->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
	world->addCollisionObject(ghostObject, COL_GHOST, COL_BOX | COL_GHOST);

	charCon = new btKinematicCharacterController(ghostObject, ghostShape, 0.35f);
	charCon->setGravity(btVector3(0, -100, 0));
	charCon->setMaxJumpHeight(20.0);

	world->addAction(charCon);
}

void setCursortoMiddle(HWND hwnd) {
	RECT rect;

	GetClientRect(hwnd, &rect);
	SetCursorPos(rect.right / 2, rect.bottom / 2);
}

void enableVerticalSync(bool enableVerticalSync) {
	// WGL_EXT_swap_control.
	typedef BOOL(WINAPI * PFNWGLSWAPINTERVALEXTPROC)(GLint);

	static PFNWGLSWAPINTERVALEXTPROC wglSwapIntervalEXT =
		reinterpret_cast<PFNWGLSWAPINTERVALEXTPROC>(
			wglGetProcAddress("wglSwapIntervalEXT"));

	if (wglSwapIntervalEXT) {
		wglSwapIntervalEXT(enableVerticalSync ? 1 : 0);
		g_enableVerticalSync = enableVerticalSync;
	}
}

void enableWireframe(bool enableWireframe) {

	g_enableWireframe = enableWireframe;

	if (g_enableWireframe) {
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	}
	else {
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}
}

void processInput(HWND hWnd) {

	static UCHAR pKeyBuffer[256];
	ULONG        Direction = 0;
	POINT        CursorPos;
	float        X = 0.0f, Y = 0.0f;

	// Retrieve keyboard state
	if (!GetKeyboardState(pKeyBuffer)) return;

	// Check the relevant keys
	if (pKeyBuffer['W'] & 0xF0) Direction |= DIR_FORWARD;
	if (pKeyBuffer['S'] & 0xF0) Direction |= DIR_BACKWARD;
	if (pKeyBuffer['A'] & 0xF0) Direction |= DIR_LEFT;
	if (pKeyBuffer['D'] & 0xF0) Direction |= DIR_RIGHT;
	if (pKeyBuffer['E'] & 0xF0) Direction |= DIR_UP;
	if (pKeyBuffer['Q'] & 0xF0) Direction |= DIR_DOWN;

	// Now process the mouse (if the button is pressed)
	if (GetCapture() == hWnd) {
		// Hide the mouse pointer
		SetCursor(NULL);
		// Retrieve the cursor position
		GetCursorPos(&CursorPos);

		// Calculate mouse rotational values
		X = (float)(g_OldCursorPos.x - CursorPos.x) * 0.1;
		Y = (float)(g_OldCursorPos.y - CursorPos.y) * 0.1;

		// Reset our cursor position so we can keep going forever :)
		SetCursorPos(g_OldCursorPos.x, g_OldCursorPos.y);

		if (Direction > 0 || X != 0.0f || Y != 0.0f) {

			// Rotate camera
			if (X || Y) {
				camera.rotate(X, Y, 0.0f);

			} // End if any rotation

			if (Direction) {
				UPDATEFRAME = false;
				float dx = 0, dy = 0, dz = 0, speed = 4.3;

				if (Direction & DIR_FORWARD) dz = speed;
				if (Direction & DIR_BACKWARD) dz = -speed;
				if (Direction & DIR_LEFT) dx = -speed;
				if (Direction & DIR_RIGHT) dx = speed;
				if (Direction & DIR_UP) dy = speed;
				if (Direction & DIR_DOWN) dy = -speed;

				camera.move(dx, dy, dz);
			}

		}// End if any movement
	} // End if Captured
}


void updateDyn(HWND hWnd, double elapsedTimeSec) {

	static UCHAR pKeyBuffer[256];
	ULONG        Direction = 0;
	POINT        CursorPos;
	float        X = 0.0f, Y = 0.0f;

	// Retrieve keyboard state
	if (!GetKeyboardState(pKeyBuffer)) return;

	// Check the relevant keys
	if (pKeyBuffer[VK_UP] & 0xF0) Direction |= DIR_FORWARD;
	if (pKeyBuffer[VK_DOWN] & 0xF0) Direction |= DIR_BACKWARD;
	if (pKeyBuffer[VK_LEFT] & 0xF0) Direction |= DIR_LEFT;
	if (pKeyBuffer[VK_RIGHT] & 0xF0) Direction |= DIR_RIGHT;

	float pitch = 0.0f;
	float heading = 0.0f;

	if (GetCapture() == hWnd) {
		// hide the mouse pointer
		SetCursor(NULL);

		Vector3f fb = camera.getViewDirection();
		Vector3f rl = Vector3f::Cross(camera.getViewDirection(), camera.getCamY());
		Vector3f dir(0.0f, 0.0f, 0.0f);
	
		if (Direction || (pKeyBuffer[VK_RCONTROL] & 0xF0)) UPDATEFRAME = false;

		if ((pKeyBuffer[VK_RCONTROL] & 0xF0) && dynamicCharacterController->onGround()) {
			dynamicCharacterController->jump(btVector3(0, 1, 0), 600);
			
		}

		if (Direction & DIR_FORWARD) {
			dir += fb;
		}

		if (Direction & DIR_BACKWARD) {
			dir -= fb;
		}


		if (Direction & DIR_LEFT) {
			dir -= rl;
		}

		if (Direction & DIR_RIGHT) {
			dir += rl;
		}

		if (dynamicCharacterController->onGround() && !Direction) {
			btVector3 posDyn = dynamicCharacterController->getRigidBody()->getWorldTransform().getOrigin();
			btVector3 linVel = m_groundBody->getVelocityInLocalPoint(posDyn);
			float x = linVel.x(), z = linVel.z();
			//if(abs(x) < 0.3 ) x = 0; if(abs(z) < 0.3) z = 0;
			dynamicCharacterController->setMovementXZ(Vector2f(x, z));
			btVector3 posKin = charCon->getGhostObject()->getWorldTransform().getOrigin();
			btVector3 quadOrigin = m_groundBody->getWorldTransform().getOrigin();

		}else if (dynamicCharacterController->onGround() && Direction) {
			//btVector3 pos = dynamicCharacterController2->getRigidBody()->getCenterOfMassPosition();
			//btVector3 linVel = m_groundBody->getVelocityInLocalPoint(pos).normalized();
			//dynamicCharacterController2->setMovementXZ(Vector2f(dir[0] + linVel.x(), dir[2] + linVel.z()) * 5.0);		
			dynamicCharacterController->setMovementXZ(Vector2f(dir[0], dir[2]) * 50.0);
			
		//jumping case
		}else {

			dynamicCharacterController->setMovementXZ(Vector2f(dir[0], dir[2]) * 100.0);
		}

		if (UPDATEFRAME) {
			btTransform t;
			t = dynamicCharacterController->getRigidBody()->getWorldTransform();
			btVector3 pos = t.getOrigin();
			camera.setPosition(pos.getX(), pos.getY(), pos.getZ());
		}
	}
}

void updateKin(HWND hWnd, double elapsedTimeSec) {
	static UCHAR pKeyBuffer[256];
	ULONG        Direction = 0;
	POINT        CursorPos;
	float        X = 0.0f, Y = 0.0f;

	// Retrieve keyboard state
	if (!GetKeyboardState(pKeyBuffer)) return;

	// Check the relevant keys
	if (pKeyBuffer[VK_NUMPAD8] & 0xF0) Direction |= DIR_FORWARD2;
	if (pKeyBuffer[VK_NUMPAD5] & 0xF0) Direction |= DIR_BACKWARD2;
	if (pKeyBuffer[VK_NUMPAD4] & 0xF0) Direction |= DIR_LEFT2;
	if (pKeyBuffer[VK_NUMPAD6] & 0xF0) Direction |= DIR_RIGHT2;

	float pitch = 0.0f;
	float heading = 0.0f;

	if (GetCapture() == hWnd) {
		// hide the mouse pointer
		SetCursor(NULL);

		Vector3f fb = camera.getViewDirection();
		Vector3f rl = Vector3f::Cross(camera.getViewDirection(), camera.getCamY());
		Vector3f dirKin(0.0f, 0.0f, 0.0f);

		if (Direction || (pKeyBuffer[VK_RCONTROL] & 0xF0)) UPDATEFRAME = false;

		if ((pKeyBuffer[VK_RCONTROL] & 0xF0) && charCon->onGround()) {
			charCon->jump(btVector3(0, 60, 0));
			colPoint.active = false;
			colPoint.rad = 0.0f;
			colPoint.angle = 0.0f;
		}

		if (Direction & DIR_FORWARD2) {
			dirKin += fb;
		}

		if (Direction & DIR_BACKWARD2) {
			dirKin -= fb;
		}

		if (Direction & DIR_LEFT2) {
			dirKin -= rl;
		}

		if (Direction & DIR_RIGHT2) {
			dirKin += rl;
		}

		if (charCon->onGround() && !Direction) {
			
			if (colPoint.active) {
				btMatrix3x3 rot;
				rot.setEulerZYX(0.0, colPoint.rad, 0.0);

				//rot[0][0] = 0; rot[1][0] = 0; rot[2][0] = 0;
				rot[0][1] = 0; rot[1][1] = 0; rot[2][1] = 0;
				//rot[0][2] = 0; rot[1][2] = 0; rot[2][2] = 0;

				btVector3 tmp = rot * (colPoint.position);
				btTransform transform = charCon->getGhostObject()->getWorldTransform();
				transform.setOrigin(btVector3(tmp[0], colPoint.position[1], tmp[2]));
				charCon->getGhostObject()->setWorldTransform(transform);

				colPoint.angle = colPoint.angle + angVelY;
				colPoint.rad = (PI * colPoint.angle) / 180.0f;
			}
		}else if (charCon->onGround() && Direction) {
			
			charCon->setWalkDirection(btVector3(dirKin[0], 0, dirKin[2]) * 0.8);
			colPoint.active = false;
			colPoint.rad = 0.0f;
			colPoint.angle = 0.0f;
		//jumping case
		}else {
			charCon->setWalkDirection(btVector3(dirKin[0], 0, dirKin[2]));
			colPoint.active = false;
			colPoint.rad = 0.0f;
			colPoint.angle = 0.0f;
		}

		if (UPDATEFRAME) {
			btTransform t;
			t = dynamicCharacterController->getRigidBody()->getWorldTransform();
			btVector3 pos = t.getOrigin();
			camera.setPosition(pos.getX(), pos.getY(), pos.getZ());
		}
	}
}

void kinematicPreTickCallback(btDynamicsWorld* world, btScalar deltaTime) {
	btRigidBody* groundBody = (btRigidBody*)world->getWorldUserInfo();
	btTransform predictedTrans;

	btTransform t;
	groundBody->getMotionState()->getWorldTransform(t);
	float height = t.getOrigin().y();
	if (upward & height > 0.0f) {
		velY = -16.0f;
	upward = false;
	dynamicCharacterController->setDistanceOffset(0.1);
	}

	if (!upward & height < -400.0f) {
	velY = 16.0f;
	upward = true;
	dynamicCharacterController->setDistanceOffset(0.33);
	}

	btVector3 linearVelocity(0, velY, 0);
	btVector3 angularVelocity(0, angVelY, 0);
	btTransformUtil::integrateTransform(groundBody->getWorldTransform(), linearVelocity, angularVelocity, deltaTime, predictedTrans);

	groundBody->getMotionState()->setWorldTransform(predictedTrans);
}