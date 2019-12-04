#include <iostream>
#include "include/RobotCommunicationMRDS.h"
#include "include/MainWindow.h"

#include <curl/curl.h>
#include <QApplication>


#define WIDTH 600
#define HEIGHT 600
#define STRIDE (WIDTH * 4)

unsigned char image[STRIDE*HEIGHT];

int main(int argc, char** argv) {
    auto curl = curl_easy_init();
    std::cout << "Hello, World!" << std::endl;
//    cairo_surface_t *surface;
//    cairo_t* cr;
    QApplication app (argc, argv);
    MainWindow mw;
    mw.show();

//    surface = cairo_image_surface_create_for_data (image, CAIRO_FORMAT_ARGB32,
//                                                   WIDTH, HEIGHT, STRIDE);
//
//    cr = cairo_create (surface);
//
//    cairo_rectangle (cr, 0, 0, WIDTH, HEIGHT);
//
//
//    cairo_surface_write_to_png (surface, "spiral.png");
//
//    cairo_destroy (cr);
//
//    cairo_surface_destroy (surface);


    //RobotCommunicationMRDS rc;

   // rc.GetRequest();
    return QApplication::exec();
}