
#include "CPainter3DGui.h"
#include <GL/glut.h>
#include <math.h>

namespace Painter3D {

int xnew=0, ynew=0, xnew2=0, ynew2=0;           // soucasna pozice mysi, ze ktere se pocitaji rotace a posuny
int xold=0, yold=0, xold2=0, yold2=0;           // minula pozice mysi, ze ktere se pocitaji rotace a posuny
int zold = 20;
int znew = 20;
int xx, yy, xx2, yy2,zz;                           // bod, ve kterem se nachazi kurzor mysi
int stav;                                       // stav tlacitek mysi
int zrot = 0;
int windowWidth, windowHeight;
int xOff = 0;
int yOff = 0;
int xOffOld = 0;
int yOffOld = 0;

enum {                                          // operace, ktere se mohou provadet s mysi:
    ROTATE,                                     // rotace objektu
    TRANSLATE,                                  // posun objektu
} operation=ROTATE;

CPainter3DGui *gui;

GLfloat light_position1[] = {0.0f,0.0f,10.0f,0.0f};
GLfloat light_position2[] = {5.0f,0.0f,2.0f,0.0f};
GLfloat light_position3[] = {0.0f,5.0f,2.0f,0.0f};

void onInit(void)
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);       // barva pozadi
    glPolygonMode(GL_FRONT, GL_FILL);           // nastaveni rezimu vykresleni polygonalniho modelu
    glPolygonMode(GL_BACK, GL_FILL);            // jak pro zadni tak pro predni steny
    glDisable(GL_CULL_FACE);                    // zadne hrany ani steny se nebudou odstranovat
    glPointSize(5.0);                           // velikost vykreslovanych bodu
    glEnable(GL_POINT_SMOOTH);                  // povoleni antialiasingu bodu

    glEnable(GL_DEPTH_TEST);                    // povoleni funkce pro testovani hodnot v pameti hloubky
    glDepthFunc(GL_LESS);                       // funkce pro testovani fragmentu
    glShadeModel(GL_SMOOTH);                    // nastaveni stinovaciho rezimu

//    glLightfv(GL_LIGHT0, GL_POSITION, light_position1);      // nastaveni pozice svetla
//    glLightfv(GL_LIGHT1, GL_POSITION, light_position2);      // nastaveni pozice svetla
//    glLightfv(GL_LIGHT2, GL_POSITION, light_position3);      // nastaveni pozice svetla
//    glEnable(GL_LIGHTING);                      // globalni povoleni stinovani
//    glEnable(GL_LIGHT0);                        // povoleni prvniho svetla
//    glEnable(GL_LIGHT1);                        // povoleni prvniho svetla
//    glEnable(GL_LIGHT2);                        // povoleni prvniho svetla

}

void onResize(int w, int h)                     // argumenty w a h reprezentuji novou velikost okna
{
    glViewport(0, 0, w, h);                     // viditelna oblast pres cele okno
    windowWidth=w;                              // zapamatovat si velikost okna
    windowHeight=h;
}

void setPerspectiveProjection(void)
{
    glMatrixMode(GL_PROJECTION);                // zacatek modifikace projekcni matice
    glLoadIdentity();                           // vymazani projekcni matice (=identita)
    gluPerspective(70.0, (double)windowWidth/(double)windowHeight, 0.1f, 90.0f);// nastaveni perspektivni kamery
    glMatrixMode(GL_MODELVIEW);                 // bude se menit modelova matice
    glLoadIdentity();                           // nahrat jednotkovou matici
}

inline void setColor(const CPainters::CColor &color) {
	glColor3f((float)color.getRed()/255.0, (float)color.getGreen()/255.0, (float)color.getBlue()/255.0);
}

void drawTriangles(std::vector<CPainter3DGui::NTriangle> &triangles) {
	glBegin(GL_TRIANGLES);
	for(int i=0;i<(int)triangles.size();i++) {
		setColor(triangles[i].color);
		glVertex3f(triangles[i].p1.x, triangles[i].p1.y, triangles[i].p1.z);
		glVertex3f(triangles[i].p2.x, triangles[i].p2.y, triangles[i].p2.z);
		glVertex3f(triangles[i].p3.x, triangles[i].p3.y, triangles[i].p3.z);
	}
	glEnd();
}

void drawTrianglesColored(std::vector<CPainter3DGui::NTriangleColored> &triangles) {
	glBegin(GL_TRIANGLES);
	for(int i=0;i<(int)triangles.size();i++) {
		setColor(triangles[i].c1);
		glVertex3f(triangles[i].p1.x, triangles[i].p1.y, triangles[i].p1.z);
		setColor(triangles[i].c2);
		glVertex3f(triangles[i].p2.x, triangles[i].p2.y, triangles[i].p2.z);
		setColor(triangles[i].c3);
		glVertex3f(triangles[i].p3.x, triangles[i].p3.y, triangles[i].p3.z);
	}
	glEnd();
}

// draw points: small points as openGL vertices, large points are spheres
void drawPoints(const std::vector<CPainter3DGui::NPoint> &pts) {

	for(int i=0;i<(int)pts.size();i++) {
		if (pts[i].size > 0.1) {
			glPushMatrix();
			setColor(pts[i].color);
			glTranslatef(pts[i].p.x,pts[i].p.y,pts[i].p.z);
			glutSolidSphere(pts[i].size,10,10);
			glPopMatrix();
		} else {
			glBegin(GL_POINTS);
			setColor(pts[i].color);
			glVertex3f(pts[i].p.x,pts[i].p.y,pts[i].p.z);
			glEnd();
		}
	}
}

/** draw lines: thin lines as opengl lines, thick lines as cylinders */
void drawLinesOld(const std::vector<CPainter3DGui::NLine> &lines) {

	for(int i=0;i<(int)lines.size();i++) {
		if (0 && lines[i].width > 0.1) {
			const float vx = lines[i].p2.x-lines[i].p1.x;
			const float vy = lines[i].p2.y-lines[i].p1.y;
			float vz = lines[i].p2.z-lines[i].p1.z;
			if (vz == 0) vz = 0.001;
			float v = sqrt(vx*vx+vy*vy+vz*vz);
			if (v > 0) {
				float ax = 180.0/M_PI*acos(vz/v);
				if (vz < 0.0) ax = -ax;
				float rx = -vy*vz;
				float ry = vx*vz;
				glPushMatrix();
				glTranslatef(lines[i].p1.x,lines[i].p1.y,lines[i].p1.z);
				glRotatef(ax,rx,ry,0.0);
				setColor(lines[i].color);
				GLUquadricObj *cyl = gluNewQuadric();
				gluQuadricOrientation(cyl,GLU_OUTSIDE);
//				gluQuadricDrawStyle(cyl,GLU_LINE);
				gluQuadricDrawStyle(cyl,GLU_FILL);
				gluCylinder(cyl,lines[i].width,lines[i].width,v,7,7);
				glPopMatrix();
			}
		} else {
			glBegin(GL_LINES);
			setColor(lines[i].color);
			glVertex3f(lines[i].p1.x,lines[i].p1.y,lines[i].p1.z);
			glVertex3f(lines[i].p2.x,lines[i].p2.y,lines[i].p2.z);
			glEnd();
		}
	}
}


/** draw lines: thin lines as opengl lines, thick lines as cylinders */
void drawLines(const std::vector<CPainter3DGui::NLine> &lines) {

	for(int i=0;i<(int)lines.size();i++) {
		if (lines[i].width > 0.1) {
			const double vx = lines[i].p2.x-lines[i].p1.x;
			const double vy = lines[i].p2.y-lines[i].p1.y;
			double vz = lines[i].p2.z-lines[i].p1.z;
			double v = sqrt(vx*vx+vy*vy+vz*vz);
			double ax=0.0;
			const double zero=1e-4;
//			std::cerr << "vx="<<vx<<",vy="<<vy<<",vz="<<vz<<"\n";
			if (fabs(vz) < zero) {
				ax = 180.0/M_PI*acos(vx/v);
				if (vx <= 0.0) {
					ax = -ax;
				}
			} else {
				ax = 180.0/M_PI*acos(vz/v);
				if (vz <= 0.0 ) {
					ax = -ax;
				}
			}
			double rx = -vy*vz;
			double ry = vx*vz;
			glPushMatrix();
			glTranslated(lines[i].p1.x,lines[i].p1.y,lines[i].p1.z);
			//if (fabs(vz) < zero) {
			if (vz < 0.0) {
				glRotated(90,0,1,0.0);
				glRotated(ax,1.0,0.0,0.0);
			} else {
				glRotated(ax,rx,ry,0.0);

			}
			setColor(lines[i].color);
			GLUquadricObj *cyl = gluNewQuadric();
			gluQuadricOrientation(cyl,GLU_OUTSIDE);
//				gluQuadricDrawStyle(cyl,GLU_LINE);
			gluQuadricDrawStyle(cyl,GLU_FILL);
			gluCylinder(cyl,lines[i].width,lines[i].width,v,7,7);
			glPopMatrix();
		}
		else {
			glBegin(GL_LINES);
			setColor(lines[i].color);
			glVertex3f(lines[i].p1.x,lines[i].p1.y,lines[i].p1.z);
			glVertex3f(lines[i].p2.x,lines[i].p2.y,lines[i].p2.z);
			glEnd();
		}
	}
}


void onDisplay(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// vymazani barvoveho bufferu i pameti hloubky
    setPerspectiveProjection();                 // nastaveni perspektivni kamery
	
    glTranslatef(0.0f, 0.0f, -50.0f);           // posun objektu dale od kamery
//    glTranslatef(0.0f, 0.0f, znew);             // priblizeni ci vzdaleni objektu podle pohybu kurzoru mysi
    glTranslatef(xOff, yOff, znew);             // priblizeni ci vzdaleni objektu podle pohybu kurzoru mysi
    glRotatef(ynew, 1.0f, 0.0f, 0.0f);          // rotace objektu podle pohybu kurzoru mysi
    glRotatef(xnew, 0.0f, 1.0f, 0.0f);
    glRotatef(zrot, 0.0f, 0.0f, 1.0f);

	std::vector<CPainter3DGui::NTriangle> newTriangles;
	std::vector<CPainter3DGui::NTriangleColored> newTrianglesColored;
	std::vector<CPainter3DGui::NPoint> newPoints;
	std::vector<CPainter3DGui::NLine> newLines;

	if (gui->isNewData()) {
		newTriangles.clear();
		newTrianglesColored.clear();
		newPoints.clear();
		newLines.clear();
		gui->getNewTriangles(newTriangles);
		gui->getNewLines(newLines);
		gui->getNewPoints(newPoints);
		gui->getNewTrianglesColored(newTrianglesColored);
	}
	drawTriangles(newTriangles);
	drawTrianglesColored(newTrianglesColored);
	drawPoints(newPoints);
	drawLines(newLines);


    glFlush();                                  // provedeni a vykresleni vsech zmen
    glutSwapBuffers();                          // a prohozeni predniho a zadniho bufferu

}

void onKeyboarSpecials(int key, int x, int y) {

	switch(key) {
		case GLUT_KEY_UP:
			ynew-=5;
			glutPostRedisplay();
			break;
		case GLUT_KEY_DOWN:
			ynew+=5;
			glutPostRedisplay();
			break;
		case GLUT_KEY_LEFT:
			xnew-=5;
			glutPostRedisplay();
			break;
		case GLUT_KEY_RIGHT:
			xnew+=5;
			glutPostRedisplay();
		default:
			break;

	}

}


void onKeyboard(unsigned char key, int x, int y) {

//    if (key>='A' && key<='Z')                   // uprava velkych pismen na mala
  //      key+='a'-'A';                           // pro zjednoduseni prikazu switch

    switch (key) {
        case 27:
        case 'q':
//            exit(0);
            break;                              // pokud byla stlacena klavesa ESC, konec programu
        case 'f':
            glutFullScreen();                   // prepnuti na celou obrazovku
            break;
        case 'w':
            glutReshapeWindow(500, 500);        // prepnuti zpet do okna
            break;
        case '1':                               // prepnuti na zobrazeni pouze vrcholu
            glPolygonMode(GL_FRONT,GL_POINT);
            glPolygonMode(GL_BACK,GL_POINT);
            glutPostRedisplay();
            break;
        case '2':                               // prepnuti na zobrazeni dratoveho modelu
            glPolygonMode(GL_FRONT,GL_LINE);
            glPolygonMode(GL_BACK,GL_LINE);
            glutPostRedisplay();
            break;
        case '3':                               // prepnuti na zobrazeni vyplnenych sten
            glPolygonMode(GL_FRONT,GL_FILL);
            glPolygonMode(GL_BACK,GL_FILL);
            glutPostRedisplay();
            break;
        case '4':                               // dalsi mozny zpusob vykreslovani polygonu
            glPolygonMode(GL_FRONT,GL_FILL);
            glPolygonMode(GL_BACK,GL_LINE);
            glutPostRedisplay();
            break;
        case '5':                               // dalsi mozny zpusob vykreslovani polygonu
            glPolygonMode(GL_FRONT,GL_LINE);
            glPolygonMode(GL_BACK,GL_FILL);
            glutPostRedisplay();
            break;
        case '6':                               // dalsi mozny zpusob vykreslovani polygonu
            glPolygonMode(GL_FRONT,GL_POINT);
            glPolygonMode(GL_BACK,GL_FILL);
            glutPostRedisplay();
            break;
        case '7':                               // dalsi mozny zpusob vykreslovani polygonu
            glPolygonMode(GL_FRONT,GL_FILL);
            glPolygonMode(GL_BACK,GL_POINT);
            glutPostRedisplay();
            break;
        case '8':                               // dalsi mozny zpusob vykreslovani polygonu
            glPolygonMode(GL_FRONT,GL_POINT);
            glPolygonMode(GL_BACK,GL_LINE);
            glutPostRedisplay();
            break;
        case '9':                               // dalsi mozny zpusob vykreslovani polygonu
            glPolygonMode(GL_FRONT,GL_LINE);
            glPolygonMode(GL_BACK,GL_POINT);
            glutPostRedisplay();
            break;
		case 'r': // reset matrices
			znew = 20; zold = 0;
			xnew = 0; xold = 0;
			ynew = 0; yold = 0;
			xOff = 0; xOffOld = 0;
			yOff = 0; yOffOld = 0;
			glutPostRedisplay();
            break;
		case 's':
			znew+=1;
			glutPostRedisplay();
			break;
		case 'a':
			znew-=1;
			glutPostRedisplay();
			break;
		case 'j':
			zrot+=5;
			glutPostRedisplay();
			break;
		case 'k':
			zrot-=5;
			glutPostRedisplay();
			break;
        default: 
            break;
    }
}

void onMouseButton(int button, int state, int x, int y)
{
    if (button==GLUT_LEFT_BUTTON) {             // pri zmene stavu leveho tlacitka
        operation=ROTATE;
        if (state==GLUT_DOWN) {                 // pri stlaceni tlacitka
            xx=x;                               // zapamatovat pozici kurzoru mysi
            yy=y;
        }
        else {                                  // pri pusteni tlacitka
            xold=xnew;                          // zapamatovat novy pocatek
            yold=ynew;
        }
        glutPostRedisplay();                    // prekresleni sceny
    }
    if (button==GLUT_RIGHT_BUTTON) {
        operation=TRANSLATE;
        if (state==GLUT_DOWN) {
//			zz=y;             // pri stlaceni tlacitka zapamatovat polohu kurzoru mysi
//			xx = x;
//			yy = y;
			xOffOld = x;
			yOffOld = y;
		}
        else {
//			zold=znew;                         // pri pusteni tlacitka zapamatovat novy pocatek
//			xold = xnew;
//			yold = ynew;
//			xOff += -(x - xOffOld);
//			yOff += (y - yOffOld);
			xOffOld = x;
			yOffOld = y;
		}
        glutPostRedisplay();                    // prekresleni sceny
    }
}

void onMouseMotion(int x, int y)
{
    switch (operation) {
        case ROTATE:                            // stav rotace objektu
            xnew=xold+x-xx;                     // vypocitat novou pozici
            ynew=yold+y-yy;
//          xnew=x-xx;                     // vypocitat novou pozici
//          ynew=y-yy;
            glutPostRedisplay();                // a prekreslit scenu
            break;
        case TRANSLATE:                         // stav priblizeni/oddaleni objektu
//          znew=zold+y-zz;                     // vypocitat novou pozici
//          glutPostRedisplay();                // a prekreslit scenu
//			xnew = xold + x -xx;
//			ynew = yold + y -yy;

			xOff = (x - xOffOld);
			yOff = -(y - yOffOld);
            break;
    }
}

void onIdleFunc() {

	if (gui->isNewData()) {
		glutPostRedisplay();
	}
}

void *initGui(void *data) {

	gui = (CPainter3DGui *)data;
	int argc = 1;
	const char *argv[1] = {"ahoj"};
	glutInit(&argc, (char **)argv);                      // inicializace knihovny GLUT
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);// nastaveni predniho a zadniho barvoveho bufferu
    glutInitWindowPosition(30, 30);             // pocatecni pozice leveho horniho rohu okna
    glutInitWindowSize(500, 500);               // pocatecni velikost okna
    glutCreateWindow("CPainter3DGui");// vytvoreni okna pro kresleni
    glutDisplayFunc(onDisplay);                 // registrace funkce volane pri prekreslovani okna
    glutReshapeFunc(onResize);                  // registrace funkce volane pri zmene velikosti okna
    glutKeyboardFunc(onKeyboard);               // registrace funkce volane pri stlaceni klavesy
	glutSpecialFunc(onKeyboarSpecials);
    glutMouseFunc(onMouseButton);               // registrace funkce volane pri stlaceni ci pusteni tlacitka
    glutMotionFunc(onMouseMotion);              // registrace funkce volane pri pohybu mysi se stlacenym tlacitkem
	glutIdleFunc(onIdleFunc);
    onInit();                                   // inicializace vykreslovani
    glutMainLoop();   
}






CPainter3DGui::CPainter3DGui() : CPainter3D() {

	sem_init(&semNewData,0,1);
	sem_init(&semNewPoints,0,1);
	sem_init(&semNewLines,0,1);
	sem_init(&semNewTriangles,0,1);
	sem_init(&semNewTrianglesColored,0,1);
	sem_init(&semIsNewData,0,1);
	isNewDataF = false;
	pthread_create(&thread,NULL,&initGui,(void *)this);

}


CPainter3DGui::~CPainter3DGui() {

	pthread_join(thread,NULL);
	usleep(50000);
	newLines.clear();
	newPoints.clear();
	newTrianglesColored.clear();
	newTriangles.clear();

}

void CPainter3DGui::drawBox(const gPoint3 &position, const double rx, const double ry, const double rz,
		   const double size, const CPainters::CColor &color) {
}


void CPainter3DGui::drawPoint(const gPoint3 &position, const double size, const CPainters::CColor &color) {
	sem_wait(&semNewPoints);
	newPoints.push_back(NPoint(position,size,color));
	sem_post(&semNewPoints);
}

void CPainter3DGui::drawLine(const gPoint3 &p1, const gPoint3 &p2, const double width, const CPainters::CColor &color) {
	sem_wait(&semNewLines);
	newLines.push_back(NLine(p1,p2,width,color));
	sem_post(&semNewLines);

}

void CPainter3DGui::drawLine(const std::vector<gPoint3> &pts, const double width, const CPainters::CColor &color) {
	sem_wait(&semNewLines);
	for(int i=0;i<(int)pts.size()-1;i++) {
		newLines.push_back(NLine(pts[i],pts[i+1],width,color));
	}
	sem_post(&semNewLines);
}

void CPainter3DGui::drawTriangle(const gPoint3 &p1, const gPoint3 &p2, const gPoint3 &p3, const CPainters::CColor &color) {
	sem_wait(&semNewTriangles);
	newTriangles.push_back(NTriangle(p1,p2,p3,color));
	sem_post(&semNewTriangles);
}

void CPainter3DGui::drawTriangles(const std::vector< std::vector<gPoint3> > &triangles, const CPainters::CColor &color) {
	sem_wait(&semNewTriangles);
	for(int i=0;i<(int)triangles.size();i++) {
		newTriangles.push_back(NTriangle(triangles[i][0],triangles[i][1],triangles[0][2],color));
	}
	sem_post(&semNewTriangles);
}


void CPainter3DGui::drawTriangle(const gPoint3 &p1, const gPoint3 &p2, const gPoint3 &p3, const CPainters::CColor &c1,
			const CPainters::CColor &c2, const CPainters::CColor &c3) {
	sem_wait(&semNewTrianglesColored);
	newTrianglesColored.push_back(NTriangleColored(p1,p2,p3,c1,c2,c3));
	sem_post(&semNewTrianglesColored);

}

void CPainter3DGui::drawTriangles(const std::vector< std::vector<gPoint3> > &triangles, const CPainters::CColor &c1,
			const CPainters::CColor &c2, const CPainters::CColor &c3) {
	sem_wait(&semNewTrianglesColored);
	for (int i=0;i<(int)triangles.size();i++) {
		newTrianglesColored.push_back(NTriangleColored(triangles[i][0],triangles[i][1],triangles[0][2],c1,c2,c3));
	}
	sem_post(&semNewTrianglesColored);
}



void CPainter3DGui::getNewTriangles(std::vector<NTriangle> &triangles) {
	sem_wait(&semNewTriangles);
	for(int i=0;i<(int)newTriangles.size();i++) {
		triangles.push_back(newTriangles[i]);
	}
	sem_post(&semNewTriangles);
}

void CPainter3DGui::getNewTrianglesColored(std::vector<NTriangleColored> &triangles) {
	sem_wait(&semNewTrianglesColored);
	for(int i=0;i<(int)newTrianglesColored.size();i++) {
		triangles.push_back(newTrianglesColored[i]);
	}
	sem_post(&semNewTrianglesColored);
}


void CPainter3DGui::getNewPoints(std::vector<NPoint> &points) {

	sem_wait(&semNewPoints);
	for(int i=0;i<(int)newPoints.size();i++) {
		points.push_back(newPoints[i]);
	}
	sem_post(&semNewPoints);
}

void CPainter3DGui::getNewLines(std::vector<NLine> &lines) {
	sem_wait(&semNewLines);
	
	for(int i=0;i<(int)newLines.size();i++) {
		lines.push_back(newLines[i]);
	}

	sem_post(&semNewLines);

}

void CPainter3DGui::begin() {
	sem_wait(&semIsNewData);
	isNewDataF = false;
	sem_post(&semIsNewData);

	sem_wait(&semNewLines);
	newLines.clear();
	sem_post(&semNewLines);

	sem_wait(&semNewTriangles);
	newTriangles.clear();
	sem_post(&semNewTriangles);

	sem_wait(&semNewTrianglesColored);
	newTrianglesColored.clear();
	sem_post(&semNewTrianglesColored);

	sem_wait(&semNewPoints);
	newPoints.clear();
	sem_post(&semNewPoints);	

}

void CPainter3DGui::close() {
	sem_wait(&semIsNewData);
	isNewDataF = true;
	sem_post(&semIsNewData);	
}

bool CPainter3DGui::isNewData() {
	sem_wait(&semIsNewData);
	bool r = isNewDataF;
	sem_post(&semIsNewData);	
	return r;
}

int CPainter3DGui::getType() {
	return 1;
}


} // namespace Painter3D
