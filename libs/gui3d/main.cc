//---------------------------------------------------------------------
// Ukazkovy priklad cislo 25
// Autor: Pavel Tisnovsky
//
// V tomto programu se provede nastaveni kamery v ortogonalnim rezimu.
// Pote se vykresli jednoduche teleso ve tvaru domecku, se kterym lze
// rotovat a posouvat pomoci mysi (podle stlaceneho tlacitka mysi). 
// Domecek se poprve vykresli jako vyplneny model, ale pomoci klaves
// '0' az '9' lze provest prepnuti na jiny zpusob vykreslovani.
// Pod domeckem je vykreslena mrizka, aby se vykreslovani trosku
// zpomalilo a byl videt vyznam double-bufferingu.
// V tomto prikladu je double-buffering pouzit.
//---------------------------------------------------------------------

#include <GL/glut.h>                            // hlavickovy soubor funkci GLUTu a OpenGL
#include <stdio.h>
#include <vector>
#include <fstream>
#include <iostream>
#include "CPainters3D.h"
#include <stdlib.h>


using namespace Painter3D;
typedef CPainters::CColor CColor;

int counter = 0;
#define intro counter++; if (id == -1) std::cerr << counter << " :"<< __FUNCTION__ << "\n"; if (id != counter) return;

using namespace std;


int xnew=0, ynew=0, xnew2=0, ynew2=0;           // soucasna pozice mysi, ze ktere se pocitaji rotace a posuny
int xold=0, yold=0, xold2=0, yold2=0;           // minula pozice mysi, ze ktere se pocitaji rotace a posuny
int zold = 20;
int znew = 20;
int xx, yy, xx2, yy2,zz;                           // bod, ve kterem se nachazi kurzor mysi
int stav;                                       // stav tlacitek mysi
int windowWidth, windowHeight;

enum {                                          // operace, ktere se mohou provadet s mysi:
    ROTATE,                                     // rotace objektu
    TRANSLATE,                                  // posun objektu
} operation=ROTATE;



GLfloat light_position[]={1.0f, 1.0f, 10.0f, 0.0f}; // pozice svetla

typedef Painter3D::gPoint3 Point;
//struct Point {
//	Point(float xx, float yy, float zz):x(xx),y(yy),z(zz) {}
 //   float x,y,z;	
//};

struct Triangle {
	Triangle(const Point &a, const Point &b, const Point &c): p1(a),p2(b),p3(c) {}
	Point p1,p2,p3;
};


vector<Triangle> surface;



vector<Triangle> loadTriangles(const char *filename) {
	ifstream ifs(filename);

	float a,b,c,d,e,f,g,h,i;
	vector<Triangle> res;
	while(ifs) {
		if (ifs >> a >> b >> c >> d >> e >> f >>g >>h >>i) {
			res.push_back(Triangle(Point(a,b,c),Point(d,e,f),Point(g,h,i)));	
		}
	}
	ifs.close();
	return res;
}




//---------------------------------------------------------------------
// Funkce pro inicializaci vykreslovani
//---------------------------------------------------------------------
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

    glLightfv(GL_LIGHT0, GL_POSITION, light_position);      // nastaveni pozice svetla
   // glEnable(GL_LIGHTING);                      // globalni povoleni stinovani
   // glEnable(GL_LIGHT0);                        // povoleni prvniho svetla

}


/*
//---------------------------------------------------------------------
// Nastaveni souradneho systemu v zavislosti na velikosti okna
//---------------------------------------------------------------------
void onResize(int w, int h)                     // argumenty w a h reprezentuji novou velikost okna
{
    glViewport(0, 0, w, h);                     // viditelna oblast pres cele okno
    glMatrixMode(GL_PROJECTION);                // zacatek modifikace projekcni matice
    glLoadIdentity();                           // vymazani projekcni matice (=identita)
    glOrtho(-20, 20, -20, 20, -30, 30);         // nastaveni ortogonalni projekce
                                                // zkuste zde zmenit polohu orezavacich ploch
}
*/

//---------------------------------------------------------------------
// Nastaveni souradneho systemu v zavislosti na velikosti okna
//---------------------------------------------------------------------
void onResize(int w, int h)                     // argumenty w a h reprezentuji novou velikost okna
{
    glViewport(0, 0, w, h);                     // viditelna oblast pres cele okno
    windowWidth=w;                              // zapamatovat si velikost okna
    windowHeight=h;
}



//---------------------------------------------------------------------
// Nastaveni perspektivni projekce
//---------------------------------------------------------------------
void setPerspectiveProjection(void)
{
    glMatrixMode(GL_PROJECTION);                // zacatek modifikace projekcni matice
    glLoadIdentity();                           // vymazani projekcni matice (=identita)
    gluPerspective(70.0, (double)windowWidth/(double)windowHeight, 0.1f, 90.0f);// nastaveni perspektivni kamery
    glMatrixMode(GL_MODELVIEW);                 // bude se menit modelova matice
    glLoadIdentity();                           // nahrat jednotkovou matici
}


void drawSurface() {
	glBegin(GL_TRIANGLES);
		const float s = 3;
		for(int i=0;i<(int)surface.size();i++) {
		glColor3f(0,0,1);
			glVertex3f(s*surface[i].p1.x,s*surface[i].p1.y,s*surface[i].p1.z);
		glColor3f(0,0.5,.5);
			glVertex3f(s*surface[i].p2.x,s*surface[i].p2.y,s*surface[i].p2.z);
		glColor3f(0.8,1,.5);
			glVertex3f(s*surface[i].p3.x,s*surface[i].p3.y,s*surface[i].p3.z);
		}
    glEnd();
}


//---------------------------------------------------------------------
// Tato funkce je volana pri kazdem prekresleni okna
//---------------------------------------------------------------------
void onDisplay(void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);// vymazani barvoveho bufferu i pameti hloubky



    setPerspectiveProjection();                 // nastaveni perspektivni kamery
	
    glTranslatef(0.0f, 0.0f, -50.0f);           // posun objektu dale od kamery
    glTranslatef(0.0f, 0.0f, znew);             // priblizeni ci vzdaleni objektu podle pohybu kurzoru mysi
    glRotatef(ynew, 1.0f, 0.0f, 0.0f);          // rotace objektu podle pohybu kurzoru mysi
    glRotatef(xnew, 0.0f, 1.0f, 0.0f);
	
	glPushMatrix();
//lLoadIdentity();
	glTranslatef(0,10,5);
	glutSolidSphere(4,10,10);
	glPopMatrix();
	drawSurface();	

	
    glFlush();                                  // provedeni a vykresleni vsech zmen
    glutSwapBuffers();                          // a prohozeni predniho a zadniho bufferu

	/*
	float i;
    glClear(GL_COLOR_BUFFER_BIT);               // vymazani vsech bitovych rovin barvoveho bufferu
    glMatrixMode(GL_MODELVIEW);                 // bude se menit modelova matice
    glLoadIdentity();                           // nahrat jednotkovou matici
                                                // zde se bude provadet nastaveni kamery
    glTranslatef(xnew2/10.0f, -ynew2/10.0f, 0.0f); // posun objektu podle pohybu kurzoru mysi
    gluLookAt(4.0f, 4.0f, 18.0f,                // bod, odkud se kamera diva
              0.0f, 0.0f,  0.0f,                // bod, kam se kamera diva
              0.0f, 1.0f,  0.0f);               // poloha "stropu" ve scene
    glRotatef(ynew, 1.0f, 0.0f, 0.0f);          // rotace objektu podle pohybu kurzoru mysi
    glRotatef(xnew, 0.0f, 1.0f, 0.0f);
	*/

/*
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_LINES);
    for (i=-10.0f; i<=10.0f; i+=0.5f) {         // vykresleni mrizky pod domeckem pro zdrzeni
        glVertex3f(-10.0f, -5.0f, i);
        glVertex3f( 10.0f, -5.0f, i);
        glVertex3f(     i, -5.0f, -10.0f);
        glVertex3f(     i, -5.0f,  10.0f);
    }
    glEnd();
*/

	/*
    glBegin(GL_QUADS);                          // vykresleni otevrene krychle - sten domecku
        glColor3f(0.0f, 0.0f, 1.0f); glVertex3f(-5.0f, -5.0f, -5.0f);
        glColor3f(0.0f, 1.0f, 0.0f); glVertex3f(-5.0f, -5.0f,  5.0f);
        glColor3f(0.0f, 1.0f, 1.0f); glVertex3f( 5.0f, -5.0f,  5.0f);
        glColor3f(1.0f, 0.0f, 0.0f); glVertex3f( 5.0f, -5.0f, -5.0f);
        
        glColor3f(1.0f, 0.0f, 1.0f); glVertex3f(-5.0f,  5.0f, -5.0f);
        glColor3f(1.0f, 1.0f, 0.0f); glVertex3f(-5.0f,  5.0f,  5.0f);
        glColor3f(1.0f, 1.0f, 1.0f); glVertex3f( 5.0f,  5.0f,  5.0f);
        glColor3f(0.0f, 0.0f, 1.0f); glVertex3f( 5.0f,  5.0f, -5.0f);
        
        glColor3f(0.0f, 1.0f, 0.0f); glVertex3f(-5.0f, -5.0f, -5.0f);
        glColor3f(0.0f, 1.0f, 1.0f); glVertex3f(-5.0f, -5.0f,  5.0f);
        glColor3f(1.0f, 0.0f, 0.0f); glVertex3f(-5.0f,  5.0f,  5.0f);
        glColor3f(1.0f, 0.0f, 1.0f); glVertex3f(-5.0f,  5.0f, -5.0f);
        
        glColor3f(0.0f, 1.0f, 0.0f); glVertex3f( 5.0f, -5.0f, -5.0f);
        glColor3f(0.0f, 1.0f, 1.0f); glVertex3f( 5.0f, -5.0f,  5.0f);
        glColor3f(1.0f, 0.0f, 0.0f); glVertex3f( 5.0f,  5.0f,  5.0f);
        glColor3f(1.0f, 0.0f, 1.0f); glVertex3f( 5.0f,  5.0f, -5.0f);
    glEnd();
	*/
	/*
    glBegin(GL_TRIANGLES);                      // vykresleni strechy domecku z trojuhelniku
        glColor3f(0.0f, 0.0f, 1.0f); glVertex3f(-5.0f,  5.0f, -5.0f);
        glColor3f(0.0f, 1.0f, 1.0f); glVertex3f( 5.0f,  5.0f, -5.0f);
        glColor3f(1.0f, 1.0f, 1.0f); glVertex3f( 0.0f, 11.0f,  0.0f);

        glColor3f(1.0f, 0.0f, 0.0f); glVertex3f( 5.0f,  5.0f, -5.0f);
        glColor3f(1.0f, 1.0f, 0.0f); glVertex3f( 5.0f,  5.0f,  5.0f);
        glColor3f(1.0f, 1.0f, 1.0f); glVertex3f( 0.0f, 11.0f,  0.0f);

        glColor3f(0.0f, 1.0f, 0.0f); glVertex3f( 5.0f,  5.0f,  5.0f);
        glColor3f(0.0f, 1.0f, 1.0f); glVertex3f(-5.0f,  5.0f,  5.0f);
        glColor3f(1.0f, 1.0f, 1.0f); glVertex3f( 0.0f, 11.0f,  0.0f);

        glColor3f(0.0f, 1.0f, 0.0f); glVertex3f(-5.0f,  5.0f,  5.0f);
        glColor3f(1.0f, 1.0f, 0.0f); glVertex3f(-5.0f,  5.0f, -5.0f);
        glColor3f(1.0f, 1.0f, 1.0f); glVertex3f( 0.0f, 11.0f,  0.0f);
    glEnd();
	*/
//    glFlush();                                  // provedeni a vykresleni vsech zmen
  //  glutSwapBuffers();                          // prohozeni predniho a zadniho bufferu
}



//---------------------------------------------------------------------
// Tato funkce je volana pri stlaceni ASCII klavesy
//---------------------------------------------------------------------
void onKeyboard(unsigned char key, int x, int y)
{
    if (key>='A' && key<='Z')                   // uprava velkych pismen na mala
        key+='a'-'A';                           // pro zjednoduseni prikazu switch

    switch (key) {
        case 27:
        case 'q':
            exit(0);
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
        default: 
            break;
    }
}


//---------------------------------------------------------------------
// Tato funkce je volana pri stisku ci pusteni tlacitka mysi
//---------------------------------------------------------------------
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
        if (state==GLUT_DOWN) zz=y;             // pri stlaceni tlacitka zapamatovat polohu kurzoru mysi
        else zold=znew;                         // pri pusteni tlacitka zapamatovat novy pocatek
        glutPostRedisplay();                    // prekresleni sceny
    }
}


/*
//---------------------------------------------------------------------
// Tato funkce je volana pri stisku ci pusteni tlacitka mysi
//---------------------------------------------------------------------
void onMouseButton(int button, int state, int x, int y)
{
    if (button==GLUT_LEFT_BUTTON) {             // reakce na zmenu stavu leveho tlacitka mysi
        if (state==GLUT_DOWN) {                 // pri stlaceni
            stav=1;                             // nastaveni pro funkci motion
            xx=x;                               // zapamatovat pozici kurzoru mysi
            yy=y;
        }
        else {                                  // GLUT_UP
            stav=0;                             // normalni stav
            xold=xnew;                          // zapamatovat novy pocatek rotace
            yold=ynew;
        }
    }
    if (button==GLUT_RIGHT_BUTTON) {            // reakce na zmenu stavu praveho tlacitka mysi
        if (state==GLUT_DOWN) {                 // pri stlaceni
            stav=2;                             // nastaveni pro funkci motion
            xx2=x;                              // zapamatovat pozici kurzoru mysi
            yy2=y;
        }
        else {                                  // GLUT_UP
            stav=0;                             // normalni stav
            xold2=xnew2;                        // zapamatovat novy pocatek posunu
            yold2=ynew2;
        }
    }
    glutPostRedisplay();                        // prekresleni sceny
}
*/

/*
//---------------------------------------------------------------------
// Tato funkce je volana pri pohybu mysi se stlacenym tlacitkem
// To, ktere tlacitko je stlaceno si musime zaznamenat do glob. promenne
//---------------------------------------------------------------------
void onMouseMotion(int x, int y)
{
    if (stav==1) {                              // stav rotace objektu
        xnew=xold+x-xx;                         // vypocitat novou pozici
        ynew=yold+y-yy;
        glutPostRedisplay();                    // a prekreslit scenu
    }
    if (stav==2) {                              // stav presunu objektu
        xnew2=xold2+x-xx2;                      // vypocitat novou pozici
        ynew2=yold2+y-yy2;
        glutPostRedisplay();                    // a prekreslit scenu
    }
}
*/

//---------------------------------------------------------------------
// Tato funkce je volana pri pohybu mysi se stlacenym tlacitkem.
// To, ktere tlacitko je stlaceno si musime predem zaznamenat do
// globalni promenne stav ve funkci onMouseButton()
//---------------------------------------------------------------------
void onMouseMotion(int x, int y)
{
    switch (operation) {
        case ROTATE:                            // stav rotace objektu
            xnew=xold+x-xx;                     // vypocitat novou pozici
            ynew=yold+y-yy;
            glutPostRedisplay();                // a prekreslit scenu
            break;
        case TRANSLATE:                         // stav priblizeni/oddaleni objektu
            znew=zold+y-zz;                     // vypocitat novou pozici
            glutPostRedisplay();                // a prekreslit scenu
            break;
    }
}


void testSurfacePainting(int argc, char **argv, const int id) {
	intro;
	if (argc < 2) {
		cerr << "usage: " << argv[0] << " <rawFile>\n";
		cerr << "rawFile    3D triangles in RAW format\n";
		exit(0);
	}

	surface = loadTriangles(argv[1]);

    glutInit(&argc, argv);                      // inicializace knihovny GLUT
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);// nastaveni predniho a zadniho barvoveho bufferu
    glutInitWindowPosition(30, 30);             // pocatecni pozice leveho horniho rohu okna
    glutInitWindowSize(500, 500);               // pocatecni velikost okna
    glutCreateWindow("Priklad na OpenGL cislo 25");// vytvoreni okna pro kresleni
    glutDisplayFunc(onDisplay);                 // registrace funkce volane pri prekreslovani okna
    glutReshapeFunc(onResize);                  // registrace funkce volane pri zmene velikosti okna
    glutKeyboardFunc(onKeyboard);               // registrace funkce volane pri stlaceni klavesy
    glutMouseFunc(onMouseButton);               // registrace funkce volane pri stlaceni ci pusteni tlacitka
    glutMotionFunc(onMouseMotion);              // registrace funkce volane pri pohybu mysi se stlacenym tlacitkem
    onInit();                                   // inicializace vykreslovani
    glutMainLoop();   

}

void testGui(int argc, char **argv, const int id) {
	intro;

	if (argc < 3) {
		cerr << "usage: " << argv[0] << " <rawFile> <painterType>\n";
		cerr << "rawFile       3D triangles in RAW format\n";
		cerr << "painterType   0 .. gui\n";
		cerr << "              1 .. povray\n";
		exit(0);
	}

	const char *mapFile = argv[1];
	const int painterType = atoi(argv[2]);

	surface = loadTriangles(mapFile);

	CPainter3D *pa = NULL;
	
	if (painterType == 0) {
		pa = new CPainter3DGui();
	} else {
		pa = new CPainter3DPovray("tst_mask");
	}

	cerr << "*";
	pa->begin();
	for(int i=0;i<surface.size();i++) {
		pa->drawTriangle(surface[i].p1,surface[i].p2,surface[i].p3,CColor("red"));
	}

	pa->drawPoint(Point(5,5,5),1.4,CColor("green"));


	const double ws = 1;


	pa->drawLine(Point(0,0,0),Point(ws,0,0),0.11,CColor("red"));
	pa->drawLine(Point(0,0,0),Point(0,ws,0),0.11,CColor("green"));
	pa->drawLine(Point(0,0,0),Point(0,0,ws),0.11,CColor("blue"));


	pa->close();

	cerr << "*";
	sleep(30);

	delete pa;

}


//---------------------------------------------------------------------
// Hlavni funkce konzolove aplikace
//---------------------------------------------------------------------
int main(int argc, char **argv)
{
	const int k = argc < 2?-1:atoi(argv[1]);
	if (k > 0) {
		argc--;
		argv++;
	} else {
		std::cerr << "Choose on of the following functions:\n";
	}

	testSurfacePainting(argc,argv,k);
	testGui(argc,argv,k);
	return 0;
}



//---------------------------------------------------------------------
// Konec zdrojoveho souboru
//---------------------------------------------------------------------

