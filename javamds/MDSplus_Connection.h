/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class MDSplus_Connection */

#ifndef _Included_MDSplus_Connection
#define _Included_MDSplus_Connection
#ifdef __cplusplus
extern "C" {
#endif
/*
 * Class:     MDSplus_Connection
 * Method:    connectToMds
 * Signature: (LMDSplus/String;)I
 */
  JNIEXPORT jint JNICALL Java_MDSplus_Connection_connectToMds(JNIEnv *, jobject, jstring);

/*
 * Class:     MDSplus_Connection
 * Method:    disconnectFromMds
 * Signature: (I)V
 */
  JNIEXPORT void JNICALL Java_MDSplus_Connection_disconnectFromMds(JNIEnv *, jobject, jint);

/*
 * Class:     MDSplus_Connection
 * Method:    openTree
 * Signature: (ILjava/lang/String;I)V
 */
  JNIEXPORT void JNICALL Java_MDSplus_Connection_openTree(JNIEnv *, jobject, jint, jstring, jint);

/*
 * Class:     MDSplus_Connection
 * Method:    closeTree
 * Signature: (I)V
 */
  JNIEXPORT void JNICALL Java_MDSplus_Connection_closeTree(JNIEnv *, jobject, jint);

/*
 * Class:     MDSplus_Connection
 * Method:    setDefault
 * Signature: (ILjava/lang/String;)V
 */
  JNIEXPORT void JNICALL Java_MDSplus_Connection_setDefault(JNIEnv *, jobject, jint, jstring);

/*
 * Class:     MDSplus_Connection
 * Method:    get
 * Signature: (ILjava/lang/String;[LMDSplus/Data;)LMDSplus/Data;
 */
  JNIEXPORT jobject JNICALL Java_MDSplus_Connection_get
      (JNIEnv *, jobject, jint, jstring, jobjectArray);

/*
 * Class:     MDSplus_Connection
 * Method:    put
 * Signature: (ILjava/lang/String;Ljava/lang/String;[LMDSplus/Data;)V
 */
  JNIEXPORT void JNICALL Java_MDSplus_Connection_put
      (JNIEnv *, jobject, jint, jstring, jstring, jobjectArray);

#ifdef __cplusplus
}
#endif
#endif
