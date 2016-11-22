
/*!
***************************************************************************
*
* \file symbol.h
*
* \brief
*    Generic Symbol writing interface
*
* \date
*    18 Jan 2006
*
* \author
*    Karsten Suehring
**************************************************************************/
struct writeMB {
  void (*writeMB_typeInfo)      (Macroblock *currMB, SyntaxElement *se, DataPartition *dP);
  void (*writeIntraPredMode)    (SyntaxElement *se, DataPartition *dP);
  void (*writeB8_typeInfo)      (SyntaxElement *se, DataPartition *dP);
  void (*writeRefFrame[6])      (Macroblock *currMB, SyntaxElement *se, DataPartition *dP);
  void (*writeMVD)              (Macroblock *currMB, SyntaxElement *se, DataPartition *dP);
  void (*writeCBP)              (Macroblock* currMB, SyntaxElement *se, DataPartition *dP);
  void (*writeDquant)           (Macroblock* currMB, SyntaxElement *se, DataPartition *dP);
  void (*writeCIPredMode)       (Macroblock* currMB, SyntaxElement *se, DataPartition *dP);
  void (*writeFieldModeInfo)    (Macroblock *currMB, SyntaxElement *se, DataPartition *dP);
  void (*writeMB_transform_size)(Macroblock *currMB, SyntaxElement *se, DataPartition *dP);
};
