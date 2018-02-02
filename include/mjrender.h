//---------------------------------//
//  This file is part of MuJoCo    //
//  Written by Emo Todorov         //
//  Copyright (C) 2017 Roboti LLC  //
//---------------------------------//


#pragma once


typedef enum _mjtGridPos            // grid position for overlay
{
    mjGRID_TOPLEFT      = 0,        // top left
    mjGRID_TOPRIGHT,                // top right
    mjGRID_BOTTOMLEFT,              // bottom left
    mjGRID_BOTTOMRIGHT              // bottom right
} mjtGridPos;


typedef enum _mjtFramebuffer        // OpenGL framebuffer option
{
    mjFB_WINDOW         = 0,        // default/window buffer
    mjFB_OFFSCREEN                  // offscreen buffer
} mjtFramebuffer;


typedef enum _mjtFontScale          // font scale, used at context creation
{
    mjFONTSCALE_100     = 100,      // normal scale, suitable in the absence of DPI scaling
    mjFONTSCALE_150     = 150,      // 150% scale
    mjFONTSCALE_200     = 200       // 200% scale
} mjtFontScale;


typedef enum _mjtFont               // font type, used at each text operation
{
    mjFONT_NORMAL       = 0,        // normal font
    mjFONT_SHADOW,                  // normal font with shadow (for higher contrast)
    mjFONT_BIG                      // big font (for user alerts)
} mjtFont;


struct _mjrRect                     // OpenGL rectangle
{
    int left;                       // left (usually 0)
    int bottom;                     // bottom (usually 0)
    int width;                      // width (usually buffer width)
    int height;                     // height (usually buffer height)
};
typedef struct _mjrRect mjrRect;


struct _mjrContext                  // custom OpenGL context
{
    // parameters copied from mjVisual
    float lineWidth;                // line width for wireframe rendering
    float shadowClip;               // clipping radius for directional lights
    float shadowScale;              // fraction of light cutoff for spot lights
    int shadowSize;                 // size of shadow map texture
    int offWidth;                   // width of offscreen buffer
    int offHeight;                  // height of offscreen buffer
    int offSamples;                 // number of offscreen buffer multisamples

    // offscreen rendering objects
    unsigned int offFBO;            // offscreen framebuffer object
    unsigned int offFBO_r;          // offscreen framebuffer for resolving multisamples
    unsigned int offColor;          // offscreen color buffer
    unsigned int offColor_r;        // offscreen color buffer for resolving multisamples
    unsigned int offDepthStencil;   // offscreen depth and stencil buffer
    unsigned int offDepthStencil_r; // offscreen depth and stencil buffer for resolving multisamples

    // shadow rendering objects
    unsigned int shadowFBO;         // shadow map framebuffer object
    unsigned int shadowTex;         // shadow map texture

    // texture objects and info
    int ntexture;                   // number of allocated textures
    int textureType[100];           // type of texture (mjtTexture)
    unsigned int texture[100];      // texture names

    // displaylist starting positions
    unsigned int basePlane;         // all planes from model
    unsigned int baseMesh;          // all meshes from model
    unsigned int baseHField;        // all hfields from model
    unsigned int baseBuiltin;       // all buildin geoms, with quality from model
    unsigned int baseFontNormal;    // normal font
    unsigned int baseFontShadow;    // shadow font
    unsigned int baseFontBig;       // big font

    // displaylist ranges
    int     rangePlane;             // all planes from model
    int     rangeMesh;              // all meshes from model
    int     rangeHField;            // all hfields from model
    int     rangeBuiltin;           // all builtin geoms, with quality from model
    int     rangeFont;              // all characters in font

    // character info
    int     charWidth[127];         // character widths: normal and shadow
    int     charWidthBig[127];      // chacarter widths: big
    int     charHeight;             // character heights: normal and shadow
    int     charHeightBig;          // character heights: big

    // capabilities
    int     glewInitialized;        // is glew initialized
    int     windowAvailable;        // is default/window framebuffer available
    int     windowSamples;          // number of samples for default/window framebuffer
    int     windowStereo;           // is stereo available for default/window framebuffer
    int     windowDoublebuffer;     // is default/window framebuffer double buffered

    // only field that changes after mjr_makeContext
    int     currentBuffer;          // currently active framebuffer: mjFB_WINDOW or mjFB_OFFSCREEN
};
typedef struct _mjrContext mjrContext;
