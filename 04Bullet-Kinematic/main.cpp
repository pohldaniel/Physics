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

int height = 960;
int width = 1280;

POINT g_OldCursorPos;
bool g_enableVerticalSync;
bool g_enableWireframe;

enum collisiontypes{
	COL_NOTHING = 0,  
	COL_SPHERE1 = 1,  
	COL_SPHERE2 = 2, 
	COL_GHOST   = 4, 
	COL_BOX		= 8,  

	COL_FORCE_8BIT = 0xFF
};

enum DIRECTION {
	DIR_FORWARD = 1,
	DIR_BACKWARD = 2,
	DIR_LEFT = 4,
	DIR_RIGHT = 8,
	DIR_UP = 16,
	DIR_DOWN = 32,

	DIR_FORCE_32BIT = 0x7FFFFFFF
};

SkyBox* skyBox;
Camera camera;
MeshSphere *sphere;
MeshQuad *quad;
bool UPDATEFRAME = true;
double delta_Time;

btDynamicsWorld* world;	//every physical object go to the world
btDispatcher* dispatcher;	//what collision algorithm to use?
btCollisionConfiguration* collisionConfig;	//what collision algorithm to use?
btBroadphaseInterface* broadphase;	//should Bullet examine every object, or just what close to each other
btConstraintSolver* solver;					//solve collisions, apply forces, impulses
std::vector<btRigidBody*> bodies;

btKinematicCharacterController* charCon;
btPairCachingGhostObject* ghostObject;

btRigidBody* addSphere(float rad, float x, float y, float z, float mass);
void renderSphere(btRigidBody* sphere);
void renderPlane(btRigidBody* plane);
void renderBox(btRigidBody* box);
//prototype funktions
LRESULT CALLBACK winProc(HWND hWnd, UINT message, WPARAM wParma, LPARAM lParam);
void setCursortoMiddle(HWND hwnd);
void enableWireframe(bool enableWireframe);
void enableVerticalSync(bool enableVerticalSync);

void initApp(HWND hWnd);
void processInput(HWND hWnd);
void updateFrame(HWND hWnd, double elapsedTimeSec);
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
						
			world->stepSimulation(1.0 / 60.0);

			for (int i = 0; i<bodies.size(); i++){
				if (bodies[i]->getCollisionShape()->getShapeType() == SPHERE_SHAPE_PROXYTYPE) 
					renderSphere(bodies[i]);
				else if (bodies[i]->getCollisionShape()->getShapeType() == BOX_SHAPE_PROXYTYPE)
					renderBox(bodies[i]);
			}

			
			for (int i = 0; i < ghostObject->getNumOverlappingObjects(); i++) {
				btRigidBody *pRigidBody = dynamic_cast<btRigidBody *>(ghostObject->getOverlappingObject(i));
				if (pRigidBody) {					
					pRigidBody->applyCentralImpulse(btVector3(0.0f, 10.0f, 0.0f));
				}
			}

			skyBox->draw(camera);

			processInput(hwnd);
			updateFrame(hwnd, delta_Time);

			hdc = GetDC(hwnd);
			SwapBuffers(hdc);
			ReleaseDC(hwnd, hdc);
		}
	} // end while

	delete sphere;
	delete quad;
	delete skyBox;

	for (int i = 0; i<bodies.size(); i++){
		world->removeCollisionObject(bodies[i]);
		btMotionState* motionState = bodies[i]->getMotionState();
		btCollisionShape* shape = bodies[i]->getCollisionShape();
		delete bodies[i];
		delete shape;
		delete motionState;
	}
	delete dispatcher;
	delete collisionConfig;
	delete solver;
	//delete broadphase;
	
	btCollisionShape* shape = ghostObject->getCollisionShape();
	delete ghostObject;
	delete shape;	
	delete charCon;
	delete world;
	
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

		if (GetCapture() == hWnd) {
			Vector3f pos = camera.getPosition();
			Vector3f dir = camera.getViewDirection() * 200;
			btRigidBody* sphere = addSphere(20.0, pos[0], pos[1], pos[2], 1.0);
			sphere->setLinearVelocity(btVector3(dir[0], dir[1], dir[2]));
		}

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
	quad = new MeshQuad(1024, 1024, ".\\res\\floor_color_map.png");
	quad->setPrecision(1, 1);
	quad->buildMesh();

	sphere = new MeshSphere(Vector3f(0.0f, 0.0f, 0.0), 20.0f, ".\\res\\earth2048.png");
	sphere->setPrecision(20, 20);
	sphere->buildMesh();

	//pretty much initialize everything logically
	collisionConfig = new btDefaultCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfig);
	broadphase = new btDbvtBroadphase();
	solver = new btSequentialImpulseConstraintSolver();
	world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfig);
	world->setGravity(btVector3(0, -100, 0));	//gravity on Earth

	//create btPlane
	/*btTransform transPlane;
	transPlane.setIdentity();
	transPlane.setOrigin(btVector3(0, 0, 0));
	btStaticPlaneShape* btPlane = new btStaticPlaneShape(btVector3(0, 1, 0), 0);
	btMotionState* motionPlane = new btDefaultMotionState(transPlane);
	btRigidBody::btRigidBodyConstructionInfo infoPlane(0.0, motionPlane, btPlane);
	btRigidBody* bodyPlane = new btRigidBody(infoPlane);
	world->addRigidBody(bodyPlane);
	bodies.push_back(bodyPlane);*/

	//create btBox
	btTransform transBox;
	transBox.setIdentity();
	transBox.setOrigin(btVector3(0, 0, 0));
	btBoxShape* btBox = new btBoxShape(btVector3(1024.0f / 2.0, 0.0, 1024.0f / 2.0));
	btMotionState* motionBox = new btDefaultMotionState(transBox);
	btRigidBody::btRigidBodyConstructionInfo infoBox(0.0, motionBox, btBox);
	btRigidBody* bodyBox = new btRigidBody(infoBox);	
	bodyBox->setCollisionFlags(btCollisionObject::CF_STATIC_OBJECT); //determine the internal collision handling, this setting goes over group and mask setting
	bodies.push_back(bodyBox);

	world->addCollisionObject(bodyBox, COL_BOX, COL_GHOST | COL_SPHERE1 | COL_SPHERE2); //determine wich group collides with wich group  "rigidBody | group | collides with"
	
	
	//create btSpehere1
	btTransform transS1;
	transS1.setIdentity();
	transS1.setOrigin(btVector3(30.0, 20.0, 0.0));
	btSphereShape* shapeS1 = new btSphereShape(20.0);
	btVector3 inertiaS1(0, 0, 0);
	shapeS1->calculateLocalInertia(1.0, inertiaS1);
	btMotionState* motionS1 = new btDefaultMotionState(transS1);
	btRigidBody::btRigidBodyConstructionInfo infoS1(3.0, motionS1, shapeS1, inertiaS1);
	btRigidBody* bodyS1 = new btRigidBody(infoS1);
	bodyS1->setCollisionFlags(btCollisionObject::CF_DYNAMIC_OBJECT);
	bodies.push_back(bodyS1);

	world->addRigidBody(bodyS1, COL_SPHERE1, COL_BOX | COL_SPHERE1 | COL_SPHERE2 | COL_GHOST);

	//create btSpehere2
	btTransform transS2;
	transS2.setIdentity();
	transS2.setOrigin(btVector3(-30.0, 20.0, 0.0));
	btSphereShape* shapeS2 = new btSphereShape(20.0);
	btVector3 inertiaS2(0, 0, 0);
	shapeS2->calculateLocalInertia(1.0, inertiaS2);
	btMotionState* motionS2 = new btDefaultMotionState(transS2);
	btRigidBody::btRigidBodyConstructionInfo infoS2(1.0, motionS2, shapeS2, inertiaS2);
	btRigidBody* bodyS2 = new btRigidBody(infoS2);
	bodyS2->setCollisionFlags(btCollisionObject::CF_DYNAMIC_OBJECT);
	bodies.push_back(bodyS2);

	world->addRigidBody(bodyS2, COL_SPHERE2, COL_BOX);

	//create player
	broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());

	btTransform startTransform;
	startTransform.setIdentity();
	Vector3f pos = camera.getPosition();
	startTransform.setOrigin(btVector3(pos[0], pos[1], pos[2]));

	btConvexShape* ghostShape = new btCapsuleShape(20.0f, 5.0f);
	ghostObject = new btPairCachingGhostObject();
	ghostObject->setCollisionShape(ghostShape);
	ghostObject->setWorldTransform(startTransform);
	ghostObject->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);

	world->addCollisionObject(ghostObject, COL_GHOST, COL_BOX | COL_SPHERE1);

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
	}else {
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

void updateFrame(HWND hWnd, double elapsedTimeSec) {

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

		btTransform t;
		t = charCon->getGhostObject()->getWorldTransform();

		Vector3f fb = camera.getViewDirection();
		Vector3f rl = Vector3f::Cross(camera.getViewDirection(), camera.getCamY());
		Vector3f dir(0.0f, 0.0f, 0.0f);

		if (Direction || (pKeyBuffer[VK_RCONTROL] & 0xF0)) UPDATEFRAME = true;

		
		if (UPDATEFRAME) {

			if ((pKeyBuffer[VK_RCONTROL] & 0xF0) && charCon->onGround()) {
				charCon->jump(btVector3(0, 60, 0));
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
			
			if (charCon->onGround())
				charCon->setWalkDirection(btVector3(dir[0], 0, dir[2]));
			else
				charCon->setWalkDirection(btVector3(dir[0], 0, dir[2]) * 0.5f);
			
			btVector3 pos = t.getOrigin();
			camera.setPosition(pos.getX(), pos.getY(), pos.getZ());

		}else {
			charCon->setWalkDirection(btVector3(0, 0, 0));
		}
	}
}

btRigidBody* addSphere(float rad, float x, float y, float z, float mass){

	btTransform t;	//position and rotation
	t.setIdentity();
	t.setOrigin(btVector3(x, y, z));	//put it to x,y,z coordinates
	btSphereShape* sphere = new btSphereShape(rad);	//it's a sphere, so use sphereshape
	btVector3 inertia(0, 0, 0);	//inertia is 0,0,0 for static object, else
	if (mass != 0.0)
		sphere->calculateLocalInertia(mass, inertia);	//it can be determined by this function (for all kind of shapes)

	btMotionState* motion = new btDefaultMotionState(t);	//set the position (and motion)
	btRigidBody::btRigidBodyConstructionInfo info(mass, motion, sphere, inertia);	//create the constructioninfo, you can create multiple bodies with the same info
	btRigidBody* body = new btRigidBody(info);	//let's create the body itself
	body->setCollisionFlags(btCollisionObject::CF_DYNAMIC_OBJECT);
	world->addRigidBody(body, COL_SPHERE1, COL_BOX | COL_SPHERE1);	//and let the world know about it
	bodies.push_back(body);	//to be easier to clean, I store them a vector
	return body;
}

void renderSphere(btRigidBody* _sphere){
	if (_sphere->getCollisionShape()->getShapeType() != SPHERE_SHAPE_PROXYTYPE)	//only render, if it's a sphere
		return;
	float r = ((btSphereShape*)_sphere->getCollisionShape())->getRadius();
	btTransform t;
	_sphere->getMotionState()->getWorldTransform(t);	//get the transform
	float mat[16];
	t.getOpenGLMatrix(mat);	//OpenGL matrix stores the rotation and orientation

	sphere->setTransformation(Matrix4f(mat[0], mat[4], mat[8], mat[12],
										 mat[1], mat[5], mat[9], mat[13],
										 mat[2], mat[6], mat[10], mat[14],
										 mat[3], mat[7], mat[11], mat[15]));
	sphere->draw(camera);
}

void renderPlane(btRigidBody* _plane){
	if (_plane->getCollisionShape()->getShapeType() != STATIC_PLANE_PROXYTYPE)
		return;
	btTransform t;
	_plane->getMotionState()->getWorldTransform(t);
	float mat[16];
	t.getOpenGLMatrix(mat);

	quad->setTransformation(Matrix4f(mat[0], mat[4], mat[8], mat[12],
									   mat[1], mat[5], mat[9], mat[13],
									   mat[2], mat[6], mat[10], mat[14],
									   mat[3], mat[7], mat[11], mat[15]));
	quad->draw(camera);
}

void renderBox(btRigidBody* _box) {
	if (_box->getCollisionShape()->getShapeType() != BOX_SHAPE_PROXYTYPE)
		return;
	btTransform t;
	_box->getMotionState()->getWorldTransform(t);
	float mat[16];
	t.getOpenGLMatrix(mat);

	/*std::cout << mat[0] << "  " << mat[4] << "  " << mat[8] << "  " << mat[12] << std::endl;
	std::cout << mat[1] << "  " << mat[5] << "  " << mat[9] << "  " << mat[13] << std::endl;
	std::cout << mat[2] << "  " << mat[6] << "  " << mat[10] << "  " << mat[14] << std::endl;
	std::cout << mat[3] << "  " << mat[7] << "  " << mat[11] << "  " << mat[15] << std::endl;
	std::cout << "----------------" << std::endl;*/

	quad->setTransformation(Matrix4f(mat[0], mat[4], mat[8], mat[12],
		mat[1], mat[5], mat[9], mat[13],
		mat[2], mat[6], mat[10], mat[14],
		mat[3], mat[7], mat[11], mat[15]));
	quad->draw(camera);
}