/**
 * Project: Interactive ARAP
 * File:    main.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "include/Mesh.h"
int main(int argc, char *argv[]) {
   Mesh mesh("D:/MyDocs/Models/resources/models/cactus/cactus_small.off");
   mesh.launchViewer();

   return 0;
}

