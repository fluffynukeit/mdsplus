/* DO NOT EDIT THIS FILE - it is machine generated */
#include <jni.h>
/* Header for class jScope_LocalDataProvider */

#ifndef _Included_jScope_LocalDataProvider
#define _Included_jScope_LocalDataProvider
#ifdef __cplusplus
extern "C" {
#endif
#undef jScope_LocalDataProvider_RESAMPLE_TRESHOLD
#define jScope_LocalDataProvider_RESAMPLE_TRESHOLD 1000000000i64
#undef jScope_LocalDataProvider_MAX_PIXELS
#define jScope_LocalDataProvider_MAX_PIXELS 2000L
/*
 * Class:     jScope_LocalDataProvider
 * Method:    isSegmentedNode
 * Signature: (Ljava/lang/String;)Z
 */
  JNIEXPORT jboolean JNICALL Java_jScope_LocalDataProvider_isSegmentedNode
      (JNIEnv *, jclass, jstring);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    getSegment
 * Signature: (Ljava/lang/String;II)[B
 */
  JNIEXPORT jbyteArray JNICALL Java_jScope_LocalDataProvider_getSegment
      (JNIEnv *, jclass, jstring, jint, jint);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    getAllFrames
 * Signature: (Ljava/lang/String;II)[B
 */
  JNIEXPORT jbyteArray JNICALL Java_jScope_LocalDataProvider_getAllFrames
      (JNIEnv *, jclass, jstring, jint, jint);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    getInfo
 * Signature: (Ljava/lang/String;Z)[I
 */
  JNIEXPORT jintArray JNICALL Java_jScope_LocalDataProvider_getInfo
      (JNIEnv *, jclass, jstring, jboolean);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    getSegmentTimes
 * Signature: (Ljava/lang/String;Ljava/lang/String;FF)[F
 */
  JNIEXPORT jfloatArray JNICALL Java_jScope_LocalDataProvider_getSegmentTimes
      (JNIEnv *, jclass, jstring, jstring, jfloat, jfloat);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    getAllTimes
 * Signature: (Ljava/lang/String;Ljava/lang/String;)[F
 */
  JNIEXPORT jfloatArray JNICALL Java_jScope_LocalDataProvider_getAllTimes
      (JNIEnv *, jclass, jstring, jstring);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    getSegmentIdxs
 * Signature: (Ljava/lang/String;FF)[I
 */
  JNIEXPORT jintArray JNICALL Java_jScope_LocalDataProvider_getSegmentIdxs
      (JNIEnv *, jclass, jstring, jfloat, jfloat);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    SetEnvironmentSpecific
 * Signature: (Ljava/lang/String;Ljava/lang/String;)V
 */
  JNIEXPORT void JNICALL Java_jScope_LocalDataProvider_SetEnvironmentSpecific
      (JNIEnv *, jobject, jstring, jstring);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    UpdateNative
 * Signature: (Ljava/lang/String;J)V
 */
  JNIEXPORT void JNICALL Java_jScope_LocalDataProvider_UpdateNative
      (JNIEnv *, jobject, jstring, jlong);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    GetString
 * Signature: (Ljava/lang/String;)Ljava/lang/String;
 */
  JNIEXPORT jstring JNICALL Java_jScope_LocalDataProvider_GetString(JNIEnv *, jobject, jstring);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    GetFloatNative
 * Signature: (Ljava/lang/String;)D
 */
  JNIEXPORT jdouble JNICALL Java_jScope_LocalDataProvider_GetFloatNative
      (JNIEnv *, jobject, jstring);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    GetLongArrayNative
 * Signature: (Ljava/lang/String;)[J
 */
  JNIEXPORT jlongArray JNICALL Java_jScope_LocalDataProvider_GetLongArrayNative
      (JNIEnv *, jobject, jstring);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    GetFloatArrayNative
 * Signature: (Ljava/lang/String;)[F
 */
  JNIEXPORT jfloatArray JNICALL Java_jScope_LocalDataProvider_GetFloatArrayNative
      (JNIEnv *, jobject, jstring);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    GetDoubleArrayNative
 * Signature: (Ljava/lang/String;)[D
 */
  JNIEXPORT jdoubleArray JNICALL Java_jScope_LocalDataProvider_GetDoubleArrayNative
      (JNIEnv *, jobject, jstring);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    GetIntArray
 * Signature: (Ljava/lang/String;)[I
 */
  JNIEXPORT jintArray JNICALL Java_jScope_LocalDataProvider_GetIntArray(JNIEnv *, jobject, jstring);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    GetByteArray
 * Signature: (Ljava/lang/String;)[B
 */
  JNIEXPORT jbyteArray JNICALL Java_jScope_LocalDataProvider_GetByteArray
      (JNIEnv *, jobject, jstring);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    ErrorString
 * Signature: ()Ljava/lang/String;
 */
  JNIEXPORT jstring JNICALL Java_jScope_LocalDataProvider_ErrorString(JNIEnv *, jobject);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    registerEvent
 * Signature: (Ljava/lang/String;I)I
 */
  JNIEXPORT jint JNICALL Java_jScope_LocalDataProvider_registerEvent
      (JNIEnv *, jobject, jstring, jint);

/*
 * Class:     jScope_LocalDataProvider
 * Method:    unregisterEvent
 * Signature: (I)V
 */
  JNIEXPORT void JNICALL Java_jScope_LocalDataProvider_unregisterEvent(JNIEnv *, jobject, jint);

#ifdef __cplusplus
}
#endif
#endif
