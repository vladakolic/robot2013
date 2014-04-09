#include <stdlib.h>
#include <tr1/memory>

#include <ros/ros.h>
#include <map_creation/Map.h>
#include <odometry_data/Odometry.h>

#include <GL/glut.h>

std::tr1::shared_ptr<ros::NodeHandle> g_pN;

ros::Subscriber g_MapSub;
ros::Subscriber g_GlobalPositionSub;

/* Stores all information on the map and is replaced by every new incoming Map message. */
map_creation::Map::ConstPtr g_pMap;

const int g_nWINDOW_WIDTH = 800, g_nWINDOW_HEIGHT = 600;

GLfloat g_matWorldTMap[16] = {                   1.f,                     0.f, 0.f, 0.f,
                                                 0.f,                     1.f, 0.f, 0.f,
                                                 0.f,                     0.f, 1.f, 0.f,
                              g_nWINDOW_WIDTH * 0.5f, g_nWINDOW_HEIGHT * 0.5f, 0.f, 1.f};

void receiveGlobalPosition(const odometry_data::Odometry::ConstPtr pMsg)
{
}

void receiveMap(const map_creation::Map::ConstPtr& pMsg)
{
    ROS_INFO("Received map");
    g_pMap = map_creation::Map::ConstPtr(pMsg);

    /* Update the map */
    glutPostRedisplay();
}

void drawMap()
{
    if (g_pMap)
    {
        ROS_INFO("Map is visualized");

        glLoadIdentity();
        glClear(GL_COLOR_BUFFER_BIT);

        float fScaleX = (float) g_nWINDOW_WIDTH / g_pMap->nWidth;
        float fScaleY = (float) g_nWINDOW_HEIGHT / g_pMap->nHeight;

        // Scale the map to use the whole window space
        glScalef(fScaleX,
                 fScaleY,
                 1.0f);

        // Draw the map
        glBegin(GL_TRIANGLE_STRIP);
        for (size_t y = 0; y < g_pMap->nHeight; ++y)
            for (size_t x = 0; x < g_pMap->nWidth; ++x)
            {
                // Determine the cell value
                char nCellValue = g_pMap->gridMap[y * g_pMap->nWidth + x];
                if (nCellValue < 0)
                    // this cell was not visited yet
                    glColor3f(0.0f, 0.0f, 0.5f);
                else
                {
                    // the cell value expressed a propability of occupation, thus it should influence the color intensity
                    float fScale = 0.01f * nCellValue;
                    glColor3f(fScale, fScale, 0.0f);
                }
                /*
                 * 1 xxx 3
                 * x     x
                 * x     x
                 * 2 xxx 4
                 */
                glVertex2f((float) x, (float) (y + 1));
                glVertex2f((float) x, (float) y);
                glVertex2f((float) (x + 1), (float) (y + 1));
                glVertex2f((float) (x + 1), (float) y);
            }
        glEnd();

        // Draw the robot position
        ROS_INFO("placing robot at %d,%d", g_pMap->nRobotPositionX, g_pMap->nRobotPositionY);
        glBegin(GL_TRIANGLE_STRIP);
        glColor3f(1.0f, 0.0f, 0.0f);
        glVertex2f((float) g_pMap->nRobotPositionX, (float) (g_pMap->nRobotPositionY + 1));
        glVertex2f((float) g_pMap->nRobotPositionX, (float) g_pMap->nRobotPositionY);
        glVertex2f((float) (g_pMap->nRobotPositionX + 1), (float) (g_pMap->nRobotPositionY + 1));
        glVertex2f((float) (g_pMap->nRobotPositionX + 1), (float) g_pMap->nRobotPositionY);
        glEnd();

        glutSwapBuffers();
    }
    else
    {
        ROS_INFO("There is no map to visualize");
    }
}

void mainLoop()
{
    /*
    std::cout << g_matWorldTMap[0] << "\t" << g_matWorldTMap[1] << "\t" << g_matWorldTMap[2] << "\t" << g_matWorldTMap[3] << std::endl
              << g_matWorldTMap[4] << "\t" << g_matWorldTMap[5] << "\t" << g_matWorldTMap[6] << "\t" << g_matWorldTMap[7] << std::endl
              << g_matWorldTMap[8] << "\t" << g_matWorldTMap[9] << "\t" << g_matWorldTMap[10] << "\t" << g_matWorldTMap[11] << std::endl
              << g_matWorldTMap[12] << "\t" << g_matWorldTMap[13] << "\t" << g_matWorldTMap[14] << "\t" << g_matWorldTMap[15] << std::endl;
    */

    ros::spinOnce();
    ros::Rate(100).sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_view");
    g_pN = std::tr1::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
    
    g_MapSub = g_pN->subscribe("/map_creation/Map", 1, receiveMap);
    g_GlobalPositionSub = g_pN->subscribe("/motion/Odometry", 1, receiveGlobalPosition);

    map_creation::Map testMap;
    ROS_INFO("test map size %d", sizeof(testMap.gridMap));

    /*
     * Initialize GLUT.
     */
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

    /*
     * Initialize the window.
     */
    glutInitWindowPosition(50, 50);
    glutInitWindowSize(800, 600);
    glutCreateWindow("Map");

    /*
     * Set callback functions for GLUT events.
     */
    glutIdleFunc(mainLoop);
    glutDisplayFunc(drawMap);
    
    /*
     * Initialize drawing environment.
     */
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(0, g_nWINDOW_WIDTH, 0, g_nWINDOW_HEIGHT);
    glMatrixMode(GL_MODELVIEW);

    /*
     * Run the OpenGL main loop. Everytime OpenGL is finished it will change to Idle and let us execute ros specific 
     * functions in a custom main loop.
     */
    glutMainLoop();

    return EXIT_SUCCESS;
}
