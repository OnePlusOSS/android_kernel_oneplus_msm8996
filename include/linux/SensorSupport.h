#ifndef __SensorSupport_H__
#define __SensorSupport_H__

#define CWM_SUPPORT                         (1)
#define CWM_NOT_SUPPORT                     (0)

#define SP_ACCELEROMETER                    (CWM_SUPPORT)
#define SP_MAGNETIC_FIELD_SENSOR            (CWM_SUPPORT)
#define SP_GYROSCOPE                        (CWM_SUPPORT)
#define SP_LIGHT                            (CWM_SUPPORT)
#define SP_PROXIMITY                        (CWM_SUPPORT)
#define SP_PROXIMITY_GESTURE                (CWM_NOT_SUPPORT)
#define SP_MOTION                           (CWM_SUPPORT)
#define SP_PRESSURE                         (CWM_SUPPORT)
#define SP_TEMPERATURE                      (CWM_NOT_SUPPORT)
#define SP_AMBIENT_TEMPERATURE              (CWM_SUPPORT)
#define SP_RELATIVE_HUMIDITY                (CWM_NOT_SUPPORT)
#define SP_HEART_RATE                       (CWM_NOT_SUPPORT)
#define SP_AUDIO                            (CWM_NOT_SUPPORT)
#define SP_UV                               (CWM_NOT_SUPPORT)
#define SP_TOUCH                            (CWM_NOT_SUPPORT)
#define SP_ORIENTATION                      (CWM_SUPPORT)
#define SP_GRAVITY                          (CWM_SUPPORT)
#define SP_LINEAR_ACCELERATION              (CWM_SUPPORT)
#define SP_ROTATION_VECTOR                  (CWM_SUPPORT)
#define SP_MAGNETIC_FIELD_UNCALIBRATED      (CWM_SUPPORT)
#define SP_GYROSCOPE_UNCALIBRATED           (CWM_SUPPORT)
#define SP_GAME_ROTATION_VECTOR             (CWM_SUPPORT)
#define SP_GEOMAGNETIC_ROTATION_VECTOR      (CWM_SUPPORT)
#define SP_STEP_DETECTOR                    (CWM_SUPPORT)
#define SP_STEP_COUNTER                     (CWM_SUPPORT)
#define SP_SIGNIFICANT_MOTION               (CWM_SUPPORT)
#define SP_TILT_DETECTOR                    (CWM_NOT_SUPPORT)
#define SP_WAKE_UP                          (CWM_NOT_SUPPORT)
#define SP_GLANCE                           (CWM_NOT_SUPPORT)
#define SP_PICK_UP                          (CWM_NOT_SUPPORT)
#define SP_CYW_SHAKE                        (CWM_SUPPORT)
#define SP_CYW_TAP                          (CWM_SUPPORT)
#define SP_CYW_FLIP                         (CWM_SUPPORT)
#define SP_CYW_TWIST                        (CWM_NOT_SUPPORT)
#define SP_CYW_POCKET_MODE                  (CWM_SUPPORT)
#define SP_CYW_HAND_UP                      (CWM_SUPPORT)
#define SP_CYW_HAND_DOWN                    (CWM_SUPPORT)
#define SP_CYW_REAR_CAMERA                  (CWM_SUPPORT)
#define SP_CYW_FRONT_CAMERA                 (CWM_SUPPORT)
#define SP_CYW_PROXIMITY_SCREEN_ON          (CWM_NOT_SUPPORT)
#define SP_CYW_FALL                         (CWM_NOT_SUPPORT)
#define SP_CYW_PRIVATE_SENSOR_A             (CWM_SUPPORT)
#define SP_CYW_CONTEXT_AWARENESS            (CWM_SUPPORT)
#define SP_CYW_STATIC_DETECTOR              (CWM_SUPPORT)
#define SP_CYW_FACE_UP                      (CWM_NOT_SUPPORT)
#define SP_CYW_FACE_DOWN                    (CWM_NOT_SUPPORT)
#define SP_CYW_AIR_MOUSE                    (CWM_NOT_SUPPORT)
#define SP_CYW_WATCH_HANDUPDOWN             (CWM_NOT_SUPPORT)
#define SP_CYW_WATCH_TWIST                  (CWM_NOT_SUPPORT)
#define SP_CYW_BRING_TO_SEE                 (CWM_NOT_SUPPORT)
#define SP_CYW_AUTO_PICKUP                  (CWM_NOT_SUPPORT)

/* project customization */
#define SP_CYW_STEP_NOTIFIER                (CWM_NOT_SUPPORT)
#define SP_CYW_CONTEXT_AWARENESS_CUSTOM     (CWM_NOT_SUPPORT)
#define SP_CYW_UNWEAR_NOTIFICATION          (CWM_NOT_SUPPORT)
#define SP_CYW_PRIVATE_ANSWERING_CALL       (CWM_NOT_SUPPORT)

/*
    Internal
*/
#define SP_CYW_HAND_DETECTOR                (CWM_NOT_SUPPORT)
#define SP_CYW_PDR                          (CWM_SUPPORT)
#define SP_CYW_AIR_RECOGNITION              (CWM_NOT_SUPPORT)
#define SP_CYW_SLEEPING                     (CWM_NOT_SUPPORT)
#define SP_CYW_AUDIO_RECOGNITION            (CWM_NOT_SUPPORT)
#define SP_CYW_VIRTUAL_GYRO                 (CWM_NOT_SUPPORT)
#define SP_CYW_LOCALIZATION                 (CWM_NOT_SUPPORT)
#define SP_CYW_PDR_MOVEMENT                 (CWM_NOT_SUPPORT)
#define SP_CUSTOMALGO1                      (CWM_NOT_SUPPORT)
#define SP_CUSTOMALGO2                      (CWM_NOT_SUPPORT)
#define SP_CUSTOMIZED_PEDOMETER             (CWM_NOT_SUPPORT)

/**
*   Description:
*   Control Handle
*/

typedef enum {
    NonWakeUpHandle= 0,
    WakeUpHandle=1,
    InternalHandle=2,
    HANDLE_ID_END
} HANDLE_ID;

#define DISTANCE_CLOSE                      (0)
#define DISTANCE_FAR                        (5)
#define DISTANCE_NOCHANGE                   (-1)

/**
*   Description:
*   Define All kind of Sensors
*/
typedef enum {
#if SP_ACCELEROMETER
    emACCELEROMETER = 0,
#endif
#if SP_MAGNETIC_FIELD_SENSOR
    emMAGNETIC_FIELD_SENSOR ,
#endif
#if SP_GYROSCOPE
    emGYROSCOPE ,
#endif
#if SP_LIGHT
    emLIGHT ,
#endif
#if SP_PROXIMITY
    emPROXIMITY ,
#endif
#if SP_PROXIMITY_GESTURE
    emPROXIMITY_GESTURE,
#endif
#if SP_MOTION
    emMOTION    ,
#endif
#if SP_PRESSURE
    emPRESSURE  ,
#endif
#if SP_TEMPERATURE
    emTEMPERATURE   ,
#endif
#if SP_AMBIENT_TEMPERATURE
    emAMBIENT_TEMPERATURE   ,
#endif
#if SP_RELATIVE_HUMIDITY
    emRELATIVE_HUMIDITY ,
#endif
#if SP_HEART_RATE
    emHEART_RATE    ,
#endif
#if SP_AUDIO
    emAUDIO ,
#endif
#if SP_UV
	emUV,
#endif
#if SP_TOUCH
    emTOUGH  ,
#endif
#if SP_ORIENTATION
    emORIENTATION   ,
#endif
#if SP_GRAVITY
    emGRAVITY   ,
#endif
#if SP_LINEAR_ACCELERATION
    emLINEAR_ACCELERATION   ,
#endif
#if SP_ROTATION_VECTOR
    emROTATION_VECTOR   ,
#endif
#if SP_MAGNETIC_FIELD_UNCALIBRATED
    emMAGNETIC_FIELD_UNCALIBRATED   ,
#endif
#if SP_GYROSCOPE_UNCALIBRATED
    emGYROSCOPE_UNCALIBRATED    ,
#endif
#if SP_GAME_ROTATION_VECTOR
    emGAME_ROTATION_VECTOR  ,
#endif
#if SP_GEOMAGNETIC_ROTATION_VECTOR
    emGEOMAGNETIC_ROTATION_VECTOR   ,
#endif
#if SP_STEP_DETECTOR
    emSTEP_DETECTOR ,
#endif
#if SP_STEP_COUNTER
    emSTEP_COUNTER  ,
#endif
#if SP_SIGNIFICANT_MOTION
    emSIGNIFICANT_MOTION    ,
#endif
#if SP_TILT_DETECTOR
    emTILT_DETECTOR ,
#endif
#if SP_WAKE_UP
    emWAKE_UP   ,
#endif
#if SP_GLANCE
    emGLANCE    ,
#endif
#if SP_PICK_UP
    emPICK_UP   ,
#endif
#if SP_CYW_SHAKE
    emCYW_SHAKE ,
#endif
#if SP_CYW_TAP
    emCYW_TAP   ,
#endif
#if SP_CYW_FLIP
    emCYW_FLIP  ,
#endif
#if SP_CYW_TWIST
    emCYW_TWIST ,
#endif
#if SP_CYW_POCKET_MODE
    emCYW_POCKET_MODE   ,
#endif
#if SP_CYW_HAND_UP
    emCYW_HAND_UP   ,
#endif
#if SP_CYW_HAND_DOWN
    emCYW_HAND_DOWN ,
#endif
#if SP_CYW_REAR_CAMERA
    emCYW_REAR_CAMERA   ,
#endif
#if SP_CYW_FRONT_CAMERA
    emCYW_FRONT_CAMERA  ,
#endif
#if SP_CYW_PROXIMITY_SCREEN_ON
    emCYW_PROXIMITY_SCREEN_ON   ,
#endif
#if SP_CYW_FALL
    emCYW_FALL  ,
#endif
#if SP_CYW_PRIVATE_SENSOR_A
    emCYW_PRIVATE_SENSOR_A  ,
#endif
#if SP_CYW_CONTEXT_AWARENESS
    emCYW_CONTEXT_AWARENESS ,
#endif
#if SP_CYW_STATIC_DETECTOR
    emCYW_STATIC_DETECTOR   ,
#endif
#if SP_CYW_FACE_UP
    emCYW_FACE_UP   ,
#endif
#if SP_CYW_FACE_DOWN
    emCYW_FACE_DOWN ,
#endif
#if SP_CYW_AIR_MOUSE
    emCYW_AIR_MOUSE ,
#endif
#if SP_CYW_WATCH_HANDUPDOWN
    emCYW_WATCH_HANDUPDOWN  ,
#endif
#if SP_CYW_WATCH_TWIST
    emCYW_WATCH_TWIST  ,
#endif
#if SP_CYW_BRING_TO_SEE
    emCYW_BRING_TO_SEE  ,
#endif
#if SP_CYW_AUTO_PICKUP
    emCYW_AUTO_PICKUP   ,
#endif
#if SP_CYW_STEP_NOTIFIER
    emCYW_STEP_NOTIFIER,
#endif
#if SP_CYW_CONTEXT_AWARENESS_CUSTOM
    emCYW_CONTEXT_AWARENESS_CUSTOM,
#endif
#if SP_CYW_UNWEAR_NOTIFICATION
    emCYW_UNWEAR_NOTIFICATION,
#endif
#if SP_CYW_PRIVATE_ANSWERING_CALL
    emCYW_PRIVATE_ANSWERING_CALL,
#endif
#if SP_CYW_HAND_DETECTOR
    emCYW_HAND_DETECTOR ,
#endif
#if SP_CYW_PDR
    emCYW_PDR   ,
#endif
#if SP_CYW_AIR_RECOGNITION
    emCYW_AIR_RECOGNITION   ,
#endif
#if SP_CYW_SLEEPING
    emCYW_SLEEPING  ,
#endif
#if SP_CYW_AUDIO_RECOGNITION
    emCYW_AUDIO_RECOGNITION ,
#endif
#if SP_CYW_VIRTUAL_GYRO
    emCYW_VIRTUAL_GYRO  ,
#endif
#if SP_CYW_LOCALIZATION
    emCYW_LOCALIZATION  ,
#endif
#if SP_CYW_PDR_MOVEMENT
    emCYW_PDR_MOVEMENT ,
#endif
#if SP_CUSTOMALGO1
    emCUSTOMALGO1  ,
#endif
#if SP_CUSTOMALGO2
    emCUSTOMALGO2  ,
#endif
#if SP_CUSTOMIZED_PEDOMETER
    emCUSTOMIZED_PEDOMETER  ,
#endif
    SENSORS_ID_END
} SENSORS_ID;

/* Define for enable the SENSOR_ID in code */
#if SP_ACCELEROMETER
#define ACCELERATION                        (emACCELEROMETER)
#endif
#if SP_MAGNETIC_FIELD_SENSOR
#define MAGNETIC                            (emMAGNETIC_FIELD_SENSOR)
#endif
#if SP_GYROSCOPE
#define GYRO                                (emGYROSCOPE)
#endif
#if SP_LIGHT
#define LIGHT                               (emLIGHT)
#endif
#if SP_PROXIMITY
#define PROXIMITY                           (emPROXIMITY)
#endif
#if SP_PROXIMITY_GESTURE
#define PROXIMITY_GESTURE                   (emPROXIMITY_GESTURE)
#endif
#if SP_MOTION
#define MOTION                              (emMOTION)
#endif
#if SP_PRESSURE
#define PRESSURE                            (emPRESSURE)
#endif
#if SP_TEMPERATURE
#define TEMPERATURE                         (emTEMPERATURE)
#endif
#if SP_AMBIENT_TEMPERATURE
#define AMBIENT_TEMPERATURE                 (emAMBIENT_TEMPERATURE)
#endif
#if SP_RELATIVE_HUMIDITY
#define RELATIVE_HUMIDITY                   (emRELATIVE_HUMIDITY)
#endif
#if SP_HEART_RATE
#define HEART_RATE                          (emHEART_RATE)
#endif
#if SP_AUDIO
#define AUDIO                               (emAUDIO)
#endif
#if SP_UV
#define UV                                  (emUV)
#endif
#if SP_TOUCH
#define TOUCH                               (emTOUGH)
#endif
#if SP_ORIENTATION
#define ORIENTATION                         (emORIENTATION)
#endif
#if SP_GRAVITY
#define GRAVITY                             (emGRAVITY)
#endif
#if SP_LINEAR_ACCELERATION
#define LINEAR_ACCELERATION                 (emLINEAR_ACCELERATION)
#endif
#if SP_ROTATION_VECTOR
#define ROTATION_VECTOR                     (emROTATION_VECTOR)
#endif
#if SP_MAGNETIC_FIELD_UNCALIBRATED
#define MAGNETIC_UNCALIBRATED               (emMAGNETIC_FIELD_UNCALIBRATED)
#endif
#if SP_GYROSCOPE_UNCALIBRATED
#define GYROSCOPE_UNCALIBRATED              (emGYROSCOPE_UNCALIBRATED)
#endif
#if SP_GAME_ROTATION_VECTOR
#define GAME_ROTATION_VECTOR                (emGAME_ROTATION_VECTOR)
#endif
#if SP_GEOMAGNETIC_ROTATION_VECTOR
#define GEOMAGNETIC_ROTATION_VECTOR         (emGEOMAGNETIC_ROTATION_VECTOR)
#endif
#if SP_STEP_DETECTOR
#define STEP_DETECTOR                       (emSTEP_DETECTOR)
#endif
#if SP_STEP_COUNTER
#define STEP_COUNTER                        (emSTEP_COUNTER)
#endif
#if SP_SIGNIFICANT_MOTION
#define SIGNIFICANT_MOTION                  (emSIGNIFICANT_MOTION)
#endif
#if SP_TILT_DETECTOR
#define TILT_DETECTOR                       (emTILT_DETECTOR)
#endif
#if SP_WAKE_UP
#define WAKE_UP                             (emWAKE_UP)
#endif
#if SP_GLANCE
#define GLANCE                              (emGLANCE)
#endif
#if SP_PICK_UP
#define PICK_UP                             (emPICK_UP)
#endif
#if SP_CYW_SHAKE
#define CYW_SHAKE                           (emCYW_SHAKE)
#endif
#if SP_CYW_TAP
#define CYW_TAP                             (emCYW_TAP)
#endif
#if SP_CYW_FLIP
#define CYW_FLIP                            (emCYW_FLIP)
#endif
#if SP_CYW_TWIST
#define CYW_TWIST                           (emCYW_TWIST)
#endif
#if SP_CYW_POCKET_MODE
#define CYW_POCKET_MODE                     (emCYW_POCKET_MODE)
#endif
#if SP_CYW_HAND_UP
#define CYW_HAND_UP                         (emCYW_HAND_UP)
#endif
#if SP_CYW_HAND_DOWN
#define CYW_HAND_DOWN                       (emCYW_HAND_DOWN)
#endif
#if SP_CYW_REAR_CAMERA
#define CYW_REAR_CAMERA                     (emCYW_REAR_CAMERA)
#endif
#if SP_CYW_FRONT_CAMERA
#define CYW_FRONT_CAMERA                    (emCYW_FRONT_CAMERA)
#endif
#if SP_CYW_PROXIMITY_SCREEN_ON
#define CYW_PROXIMITY_SCREEN_ON             (emCYW_PROXIMITY_SCREEN_ON)
#endif
#if SP_CYW_FALL
#define CYW_FALL                            (emCYW_FALL)
#endif
#if SP_CYW_PRIVATE_SENSOR_A
#define CYW_PRIVATE_SENSOR_A                (emCYW_PRIVATE_SENSOR_A)
#endif
#if SP_CYW_CONTEXT_AWARENESS
#define CYW_CONTEXT_AWARENESS               (emCYW_CONTEXT_AWARENESS)
#endif
#if SP_CYW_STATIC_DETECTOR
#define CYW_STATIC_DETECTOR                 (emCYW_STATIC_DETECTOR)
#endif
#if SP_CYW_FACE_UP
#define CYW_FACE_UP                         (emCYW_FACE_UP)
#endif
#if SP_CYW_FACE_DOWN
#define CYW_FACE_DOWN                       (emCYW_FACE_DOWN)
#endif
#if SP_CYW_AIR_MOUSE
#define CYW_AIR_MOUSE                       (emCYW_AIR_MOUSE)
#endif
#if SP_CYW_WATCH_HANDUPDOWN
#define CYW_WATCH_HANDUPDOWN                (emCYW_WATCH_HANDUPDOWN)
#endif
#if SP_CYW_WATCH_TWIST
#define CYW_WATCH_TWIST                     (emCYW_WATCH_TWIST)
#endif
#if SP_CYW_BRING_TO_SEE
#define CYW_BRING_TO_SEE                    (emCYW_BRING_TO_SEE)
#endif
#if SP_CYW_AUTO_PICKUP
#define CYW_AUTO_PICKUP                     (emCYW_AUTO_PICKUP)
#endif
#if SP_CYW_STEP_NOTIFIER
#define CYW_STEP_NOTIFIER                   (emCYW_STEP_NOTIFIER)
#endif
#if SP_CYW_CONTEXT_AWARENESS_CUSTOM
#define CYW_CONTEXT_AWARENESS_CUSTOM        (emCYW_CONTEXT_AWARENESS_CUSTOM)
#endif
#if SP_CYW_UNWEAR_NOTIFICATION
#define CYW_UNWEAR_NOTIFICATION             (emCYW_UNWEAR_NOTIFICATION)
#endif
#if SP_CYW_PRIVATE_ANSWERING_CALL
#define CYW_PRIVATE_ANSWERING_CALL          (emCYW_PRIVATE_ANSWERING_CALL)
#endif
#if SP_CYW_HAND_DETECTOR
#define CYW_HAND_DETECTOR                   (emCYW_HAND_DETECTOR)
#endif
#if SP_CYW_PDR
#define CYW_PDR                             (emCYW_PDR)
#endif
#if SP_CYW_AIR_RECOGNITION
#define CYW_AIR_RECOGNITION                 (emCYW_AIR_RECOGNITION)
#endif
#if SP_CYW_SLEEPING
#define CYW_SLEEPING                        (emCYW_SLEEPING)
#endif
#if SP_CYW_AUDIO_RECOGNITION
#define CYW_AUDIO_RECOGNITION               (emCYW_AUDIO_RECOGNITION)
#endif
#if SP_CYW_VIRTUAL_GYRO
#define CYW_VIRTUAL_GYRO                    (emCYW_VIRTUAL_GYRO)
#endif
#if SP_CYW_LOCALIZATION
#define CYW_LOCALIZATION                    (emCYW_LOCALIZATION)
#endif
#if SP_CYW_PDR_MOVEMENT
#define CYW_PDR_MOVEMENT                    (emCYW_PDR_MOVEMENT)
#endif
#if SP_CUSTOMALGO1
#define CUSTOMALGO1                         (emCUSTOMALGO1)
#endif
#if SP_CUSTOMALGO2
#define CUSTOMALGO2                         (emCUSTOMALGO2)
#endif
#if SP_CUSTOMIZED_PEDOMETER
#define CUSTOMIZED_PEDOMETER                (emCUSTOMIZED_PEDOMETER)
#endif

#define STD_DRIVER_ID_END                   (ORIENTATION)
#define DRIVER_ID_END                       (ORIENTATION)


#define SPECIAL_ID_START                    (100)

typedef enum {
    TimestampSync = SPECIAL_ID_START,
    FLASH_DATA,
    META_DATA,
    MAGNETIC_UNCALIBRATED_BIAS,
    GYRO_UNCALIBRATED_BIAS,
    ERROR_MSG,
    BATCH_TIMEOUT,
    BATCH_FULL,
    ACCURACY_UPDATE,
    CALIBRATOR_UPDATE,
    MCU_REINITIAL,
    MCU_ENABLE_LIST,
    MCU_HW_ENABLE_LIST,
    MCU_INFO_START,
    MCU_INFO_DATA,
    MCU_INFO_END,
    MT_DATA_NS,
    MT_DATA_ND,
    MT_DATA_NE,
    MT_DATA_WS,
    MT_DATA_WD,
    MT_DATA_WE,
    MT_DATA_GS,
    MT_DATA_GD,
    MT_DATA_GE
}MCU_TO_CPU_EVENT_TYPE;

typedef enum {
#if SP_ACCELEROMETER
    ACCELEROMETER_WAKE_UP = SENSORS_ID_END,
#endif
#if SP_MAGNETIC_FIELD_SENSOR
    MAGNETIC_FIELD_SENSOR_WAKE_UP,
#endif
#if SP_GYROSCOPE
    GYROSCOPE_WAKE_UP,
#endif
#if SP_LIGHT
    LIGHT_WAKE_UP,
#endif
#if SP_PROXIMITY
    PROXIMITY_WAKE_UP,
#endif
#if SP_PRESSURE
    PRESSURE_WAKE_UP,
#endif
#if SP_TEMPERATURE
    TEMPERATURE_WAKE_UP,
#endif
#if SP_AMBIENT_TEMPERATURE
    AMBIENT_TEMPERATURE_WAKE_UP,
#endif
#if SP_RELATIVE_HUMIDITY
    RELATIVE_HUMIDITY_WAKE_UP,
#endif
#if SP_ORIENTATION
    ORIENTATION_WAKE_UP,
#endif
#if SP_GRAVITY
    GRAVITY_WAKE_UP,
#endif
#if SP_LINEAR_ACCELERATION
    LINEAR_ACCELERATION_WAKE_UP,
#endif
#if SP_ROTATION_VECTOR
    ROTATION_VECTOR_WAKE_UP,
#endif
#if SP_MAGNETIC_FIELD_UNCALIBRATED
    MAGNETIC_FIELD_UNCALIBRATED_WAKE_UP,
#endif
#if SP_GYROSCOPE_UNCALIBRATED
    GYROSCOPE_UNCALIBRATED_WAKE_UP,
#endif
#if SP_GAME_ROTATION_VECTOR
    GAME_ROTATION_VECTOR_WAKE_UP,
#endif
#if SP_GEOMAGNETIC_ROTATION_VECTOR
    GEOMAGNETIC_ROTATION_VECTOR_WAKE_UP,
#endif
#if SP_STEP_DETECTOR
    STEP_DETECTOR_WAKE_UP,
#endif
#if SP_STEP_COUNTER
    STEP_COUNTER_WAKE_UP,
#endif
#if SP_CYW_PRIVATE_SENSOR_A
    CYW_PRIVATE_SENSOR_A_WAKE_UP,
#endif
#if SP_CUSTOMIZED_PEDOMETER
    CUSTOMIZED_PEDOMETER_WAKE_UP,
#endif
    SENSORS_HAL_ID_END
} SENSORS_HAL_ID;

#endif /* __SensorSupport_H__ */
