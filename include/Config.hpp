#ifndef CONFIG_H
#define CONFIG_H

#define LOAD_3DPOINTS 1
#define OUTLINE_CONSTRAINT 0

#define RECORD_VIDEO 0
#define RECORD_ARVIDEO 0
#define RECORD_POINTSVIDEO 0
#define RECORD_LINESVIDEO 0
#define RECORD_F0 0
#define RECORD_RT 0

#define RECORD_FRAMEINFO 0

#define COMPUTE_RT 0
#define COMPUTE_PLANE 1

#define TRACK_WITH_IMU 0


#ifndef NDEBUG
#define ASSERT(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message << std::endl; \
            std::terminate(); \
        } \
    } while (false)
#else
    #define ASSERT(condition, message) do { } while (false)
#endif

#endif // CONFIG_H

