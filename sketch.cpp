#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <SPI.h>
#include <SD.h>

#include "SD_IO.h"

// for the Adafruit shield
#define TFT_DC 9
#define TFT_CS 10
#define TFT_RST 6
#define TFT_MOSI 51
#define TFT_MISO 50
#define TFT_CLK 52

// SD chip select pin
#define SD_CS 4

// joystick pins
#define JS_XY A0	// joystick 1: Vrx
#define JS_XZ A1	// joystick 1: Vry
#define JS_XW A2
#define JS_YZ 22	// joystick 1: SW
#define JS_YW 24
#define JS_ZW A3

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

#define MAXM 120	// max # of vertices
#define MAXF 120	// max # of faces

const int n = 4;	// # of dimensions

int nodes, edges;		// # of vertices and edges

int maxX = 2, minX = -2, maxY = 2, minY = -2;

int BACKCOLOR = ILI9341_WHITE;
int EDGECOLOR = ILI9341_BLACK;

// original and transformed points respectively
double initial[MAXM][n], pts[MAXM][n];

int tour[MAXM];				// eulerian tour

void uploadShape(const char * fileName) {
	// uploads the barycentric coordinates and eulerian tour
	// of the shape stored in the specified file

	File inFile = SD.open(fileName);
	if (inFile) {
		nodes = readInt(inFile);
		edges = readInt(inFile);

		for (int i = 0; i < nodes; ++i)
			for (int j = 0; j < n; ++j)
				initial[i][j] = readDouble(inFile);

		for (int i = 0; i <= edges; ++i)
			tour[i] = readInt(inFile);
	}
	else {
		// can't change the shape being displayed
		Serial.println("Failed to open file");
	}

	inFile.close();
}

void mult(double A[n][n], double B[][n], double C[][n], int m) {
	// multiplies matrices B and A, and overwrites C

	for (int k = 0; k < m; ++k) {
		for (int i = 0; i < n; ++i) {
		    C[k][i] = 0;
		    for (int j = 0; j < n; ++j) {
			C[k][i] += A[i][j] * B[k][j];
		    }
		}
	}
}

// maps a double to an integer value inside the range of
// the smallest dimension of the display
long map2TS(double xx) {

	long MAX = 10000;
	long x = (long)(xx * MAX - 1000);

	return map(x, -MAX, MAX, 10, 240);
}

// projects a mXn matrix of m n-dimensional points to a 2-dim plane
void project(double A[][n], int m) {
    
	for (int i = 0; i < m; ++i) {

		double x = 2 * A[i][0] / (A[i][2] + 2);
		double y = 2 * A[i][1] / (A[i][2] + 2);

		A[i][0] = x;
		A[i][1] = y;
	}
}

// rotates the original barycentric coordinates in 4-space
void rotate(double alpha, double beta, double gamma, double phi, 
		double mu, double theta) {

	// rotation matrix for the XY and XZ planes
	double A[n][n] = {{cos(alpha) * cos(beta), -sin(alpha), cos(alpha) * sin(beta), 0},
			{cos(beta) * sin(alpha), cos(alpha), sin(alpha) * sin(beta), 0},
			{-sin(beta), 0, cos(beta), 0},
			{0, 0, 0, 1}};

	// rotation matrix for the XW and XZ planes
	double B[n][n] = {{cos(mu), sin(mu)*sin(phi), 0, cos(phi)*sin(mu)},
			{0, cos(phi), 0, -sin(phi)},
			{0, 0, 1, 0},
			{-sin(mu), cos(mu)*sin(phi), 0, cos(mu)*cos(phi)}};


	// rotation matrix for the YZ and ZW planes
	double C[n][n] = {{1, 0, 0, 0},
			{0, cos(gamma), -cos(theta)*sin(gamma), sin(theta)*sin(gamma)},
			{0, sin(gamma), cos(gamma)*cos(theta), -cos(gamma)*sin(theta)},
			{0, 0, sin(theta), cos(theta)}};
   
	double tmp[n][n];

	// store A*B in tmp
	mult(A, B, tmp, n);
	
	// overwrite A with (A*B)*C
	mult(C, tmp, A, n);

	// store A*B*C*initial in pts
	mult(A, initial, pts, nodes);

	// map the 4d points to 2d using a perspective projection
	project(pts, nodes);
}

// draw the wireframe outline of the shape
void draw(int color) {
	for (int i = 1; i <= edges; ++i) {
		int u, v;
                // the shape is a regular 4-polytope and is eulerian
                // so draw the edges following an eulerian tour
                u = tour[i-1], v = tour[i];

		// draw a line from vertex u to vertex v
		tft.drawLine(map2TS(pts[u][0]), map2TS(pts[u][1]),
			map2TS(pts[v][0]), map2TS(pts[v][1]), color);
	}
}

void loop() {

        double dxy, dxz, dxw = 0, dyz, dyw = 0, dzw = 0;
	double xy = 0, xz = 0, xw = 0, yz = 0, yw = 0, zw = 0;

	// threshold that the input from the joysticks must exceed
        double thresh = PI / 180;

	int delta = 10;	// max change in angle
        while (true) {

		if (Serial.available()) { // there's input from the user!
			
			// clear screen
			tft.fillScreen(ILI9341_WHITE);
			
			//checking for the input from the user
			char shape = Serial.read();
			Serial.println(shape);

			if (shape == 'O') {	
				uploadShape("ortho.txt");
			}
			else if (shape == 'S') {
				uploadShape("simplex.txt");
			}
			else {
				// default shape for incorrect input
				uploadShape("cube.txt");
			}
	
			rotate(0, 0, 0, 0, 0, 0);
			draw(ILI9341_BLUE);

			Serial.print("Enter a shape: ");
		}

		// read change in angles from joysticks
   		dxy = map(analogRead(JS_XY), 0, 1024, -delta, delta) * PI / 180;
		
		dxz = map(analogRead(JS_XZ), 0, 1024, -delta, delta) * PI / 180;

		dxw = map(analogRead(JS_XW), 0, 1024, -delta, delta) * PI / 180;

		dyz = digitalRead(JS_YZ) ? 0 : delta;

		dyw = digitalRead(JS_YW) ? 0 : delta;

		dzw = map(analogRead(JS_ZW), 0, 1024, -delta, delta) * PI / 180;

		if (dxy*dxy + dxz*dxz + dxw*dxw + dyz*dyz + dyw*dyw
			+ dzw*dzw < thresh) continue;

		// update direction angles
		xy += dxy, xz += dxz, xw += dxw;
		yz += dyz, yw += dyw;
		zw += dzw;

		// draw over previous outline
		draw(BACKCOLOR);
		rotate(xy, xz, xw, yz, yw, zw);

		// draw new outline
		draw(EDGECOLOR);

		// wait until user can see the complete shape
		delay(10);
        }
}

void setup() {

	init();
	Serial.begin(9600);
	
	tft.begin();

	delay(10);

	Serial.print("Initializing SD card...");
	if (!SD.begin(SD_CS)) {
		Serial.println("failed!");
	}
	Serial.println("OK!");
	
	// initialize the joystick switches
	pinMode(JS_YZ, INPUT_PULLUP);
	pinMode(JS_YW, INPUT_PULLUP);

	// clear screen
	tft.fillScreen(BACKCOLOR);

	// upload the initial shape, the hypercube
	uploadShape("cube.txt");

	rotate(0, 0, 0, 0, 0, 0);

	draw(ILI9341_BLUE); 

	Serial.println("Enter 'O' for Orthoplex, 'S' for Simplex, 'C' for Hypercube");
	Serial.print("Enter a shape: ");
}

int main() {

	setup();

	while (true) {
		loop();
	}
	return 0;
}
