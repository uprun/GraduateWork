This is my graduate work using OpenCV library and Levenberg-Markvardt (lmfit) library to 
reconstruct 3D scene from two images
This project is for Visual Studio 2010.
To compile this project you will need OpenCV library version 2.4.8 installed on your PC.
Also you will need to set environment variables, follow this OpenCV tutorial: 
http://docs.opencv.org/doc/tutorials/introduction/windows_install/windows_install.html#set-the-opencv-enviroment-variable-and-add-it-to-the-systems-path
After you comple this project you can run it and press:
1) L - for left image from web-cam
2) R - for right image from web-cam
3) S - to save this two images on disk
4) T - for starting process of reconstruction

After you have prepared two images, you can just load them by pressing O and then T - for processing.
After algorithm finish you can find "currentPLYExportFile.ply" file on
 disk which contains points of reconstructed scene.
You can open this file using MeshLab or any other program which allow to import/open PLY file format.
