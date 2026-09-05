//======================================================================================
// msound.h - speach functions
//======================================================================================
#ifndef MSOUND_H
#define MSOUND_H

// Define byte as unsigned char for compatibility, avoiding std::byte conflict
#ifndef BYTE_TYPE
#define BYTE_TYPE unsigned char
#endif

extern void MSoundRelease(void);
extern void MSound(BYTE_TYPE cNotes, ...);

#endif