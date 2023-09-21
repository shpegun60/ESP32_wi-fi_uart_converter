#ifndef CONVERT_H
#define CONVERT_H

#include "stdint.h"
#include "stddef.h"

// types mapping -----------------------------
#define VOID_TYPE 			((unsigned char) 0)
// unsigned type
#define UINT8_TYPE  		((unsigned char) 1)
#define UINT16_TYPE 		((unsigned char) 2)
#define UINT32_TYPE 		((unsigned char) 3)
#define UINT64_TYPE 		((unsigned char) 4)
//signed type
#define CHAR_TYPE 			((unsigned char) 5)
#define INT8_TYPE 			((unsigned char) 6)
#define INT16_TYPE 			((unsigned char) 7)
#define INT32_TYPE 			((unsigned char) 8)
#define INT64_TYPE 			((unsigned char) 9)
// floating point types
#define FLOAT_TYPE 			((unsigned char) 10)
#define DOUBLE_TYPE 		((unsigned char) 11)
#define LONG_DOUBLE_TYPE 	((unsigned char) 12)
// other types
#define BOOL_TYPE 			((unsigned char) 13)
#define UINT24_TYPE 		((unsigned char) 14)
#define INT24_TYPE 			((unsigned char) 15)
#define TYPE_ARRAY_LENGTH   ((unsigned char)(INT24_TYPE + 1))
//------------------------------------

#define CONVERT_MSB ((unsigned char)0x01)
#define CONVERT_LSB ((unsigned char)0x00)


// all types. Commit this if types defines upper--------------------------

typedef void    u0;
typedef unsigned char  u8;
typedef uint16_t u16;
typedef uint32_t u24;
typedef uint32_t u32;
typedef uint64_t u64;

typedef char    c;
typedef int8_t  i8;
typedef int16_t  i16;
typedef int32_t  i24;
typedef int32_t  i32;
typedef int64_t  i64;

typedef float       f32;
typedef double      f64;
typedef long double f128;

typedef bool        b;


// bus types defining ----------------------------------------------
typedef size_t reg;  // entity pointer type
#define CONVERT_PERIF_BUS_TYPE_IND UINT32_TYPE
//------------------------------------------------------------------


#define CONVERT_USER_WRITE(to, value, type, pos, direction)\
    do{\
        Convert::userWrite((to), (value), (sizeof(type)), (pos), (direction))\
    } while(0L)

#define CONVERT_USER_READ(from, ReadedValue, type, pos, direction)\
    do{\
        Convert::userRead((from), (ReadedValue), (sizeof(type)), (pos), (direction));\
    } while(0L)

class Convert
{
private:

    /*
     * ***********************************************************************************************************************************
     *  user overwrite expressions
     * ***********************************************************************************************************************************
     */
    static inline void pointerCopy(unsigned int n, unsigned char *fromPtr, unsigned char *toPtr, unsigned int *pos, unsigned char direction) {

        if(direction) {
            while(n--) {
                *toPtr++ = *(fromPtr + n);
                (*pos)++;
            }
        } else {
            while(n--) {
                *toPtr++ = *fromPtr++;
                (*pos)++;
            }
        }
    }

    // ***********************************************************************************************************************************

public:
    static unsigned char getTypeLen(unsigned char type);

    static inline void userWrite(unsigned char * to, unsigned char *value, unsigned int size, unsigned int *pos, unsigned char direction) {pointerCopy(size, value, (unsigned char *)(to + (*pos)), pos, direction);}
    static inline void userWrite(unsigned char * to, unsigned char *value, unsigned int size, unsigned int pos, unsigned char direction) {pointerCopy(size, value, (unsigned char *)(to + (pos)), &pos, direction);}

    static inline void userRead(unsigned char *from, unsigned char *ReadedValue, unsigned int size, unsigned int *pos, unsigned char direction) {pointerCopy(size, (unsigned char *)(from + (*pos)), ReadedValue, pos, direction);}
    static inline void userRead(unsigned char *from, unsigned char *ReadedValue, unsigned int size, unsigned int pos, unsigned char direction) {pointerCopy(size, (unsigned char *)(from + (pos)), ReadedValue, &pos, direction);}


    struct FB {

        // general functions------------------------------------------------------------------
        template <class T>
        static T  read(const unsigned char *a, unsigned int *pos);
        template <class T>
        static T  read(const unsigned char *a, unsigned int pos);
        template <class T>
        static T readCheck(const unsigned char *a, unsigned int *pos, unsigned int bufSize);
        template <class T>
        static T readCheck(const unsigned char *a, unsigned int pos, unsigned int bufSize);

        template <class T>
        static void write(const unsigned char *a, unsigned int *pos, T val);
        template <class T>
        static void write(const unsigned char *a, unsigned int pos, T val);
        template <class T>
        static void writeCheck(const unsigned char *a, unsigned int *pos, unsigned int bufSize, T val);
        template <class T>
        static void writeCheck(const unsigned char *a, unsigned int pos, unsigned int bufSize, T val);



        // individual functions---------------------------------------------------------------
        inline static u8 readU8(const unsigned char *a, unsigned int *pos)                             {return read<u8>(a, pos);}
        inline static void writeU8(const unsigned char *a, unsigned int *pos, u8 val)                  {write<u8>(a, pos, val);}
        inline static i8 readI8(const unsigned char *a, unsigned int *pos)                             {return read<i8>(a, pos);}
        inline static void writeI8(const unsigned char *a, unsigned int *pos, i8 val)                  {write<i8>(a, pos, val);}

        inline static u16 readU16(const unsigned char *a, unsigned int *pos)                           {return read<u16>(a, pos);}
        inline static void writeU16(const unsigned char *a, unsigned int *pos, u16 val)                {write<u16>(a, pos, val);}
        inline static i16 readI16(const unsigned char *a, unsigned int *pos)                           {return read<i16>(a, pos);}
        inline static void writeI16(const unsigned char *a, unsigned int *pos, i16 val)                {write<i16>(a, pos, val);}

        inline static u24 readU24(const unsigned char *a, unsigned int *pos)                           {u24 value = 0; pointerCopy(0x03U, (unsigned char *)(a + (*pos)), (unsigned char *)&value, pos, CONVERT_MSB); return (value & 0x00FFFFFFU);}
        inline static void writeU24(const unsigned char *a, unsigned int *pos, u24 val)                {pointerCopy(0x03U, (unsigned char *)&val, (unsigned char *)(a + (*pos)), pos, CONVERT_MSB);}
        inline static i24 readI24(const unsigned char *a, unsigned int *pos)                           {
            i24 value = 0; 
            pointerCopy(0x03U, (unsigned char *)(a + (*pos)), (unsigned char *)&value, pos, CONVERT_MSB); 
            value = (value & 0x00800000U) ? (value | 0xFF000000U) : (value & 0x00FFFFFFU);
            return value;
        }
        inline static void writeI24(const unsigned char *a, unsigned int *pos, u24 val)                {pointerCopy(0x03U, (unsigned char *)&val, (unsigned char *)(a + (*pos)), pos, CONVERT_MSB);}

        inline static u32 readU32(const unsigned char *a, unsigned int *pos)                           {return read<u32>(a, pos);}
        inline static void writeU32(const unsigned char *a, unsigned int *pos, u32 val)                {write<u32>(a, pos, val);}
        inline static i32 readI32(const unsigned char *a, unsigned int *pos)                           {return read<i32>(a, pos);}
        inline static void writeI32(const unsigned char *a, unsigned int *pos, i32 val)                {write<i32>(a, pos, val);}

        inline static u64 readU64(const unsigned char *a, unsigned int *pos)                           {return read<u64>(a, pos);}
        inline static void writeU64(const unsigned char *a, unsigned int *pos, u64 val)                {write<u64>(a, pos, val);}
        inline static i64 readI64(const unsigned char *a, unsigned int *pos)                           {return read<i64>(a, pos);}
        inline static void writeI64(const unsigned char *a, unsigned int *pos, i64 val)                {write<i64>(a, pos, val);}


        inline static f32 readFloat(const unsigned char *a, unsigned int *pos)                         {return read<f32>(a, pos);}
        inline static void writeFloat(const unsigned char *a, unsigned int *pos, f32 val)              {write<f32>(a, pos, val);}

        inline static f64 readDouble(const unsigned char *a, unsigned int *pos)                        {return read<f64>(a, pos);}
        inline static void writeDouble(const unsigned char *a, unsigned int *pos, f64 val)             {write<f64>(a, pos, val);}

        inline static f128 readLongDouble(const unsigned char *a, unsigned int *pos)                   {return read<f128>(a, pos);}
        inline static void writeLongDouble(const unsigned char *a, unsigned int *pos, f128 val)        {write<f128>(a, pos, val);}
    };



    struct FL {

        // general functions------------------------------------------------------------------
        template <class T>
        static T  read(const unsigned char *a, unsigned int *pos);
        template <class T>
        static T  read(const unsigned char *a, unsigned int pos);
        template <class T>
        static T readCheck(const unsigned char *a, unsigned int *pos, unsigned int bufSize);
        template <class T>
        static T readCheck(const unsigned char *a, unsigned int pos, unsigned int bufSize);

        template <class T>
        static void write(const unsigned char *a, unsigned int *pos, T val);
        template <class T>
        static void write(const unsigned char *a, unsigned int pos, T val);
        template <class T>
        static void writeCheck(const unsigned char *a, unsigned int *pos, unsigned int bufSize, T val);
        template <class T>
        static void writeCheck(const unsigned char *a, unsigned int pos, unsigned int bufSize, T val);
        


        // individual functions---------------------------------------------------------------
        inline static u8 readU8(const unsigned char *a, unsigned int *pos)                             {return read<u8>(a, pos);}
        inline static void writeU8(const unsigned char *a, unsigned int *pos, u8 val)                  {write<u8>(a, pos, val);}
        inline static i8 readI8(const unsigned char *a, unsigned int *pos)                             {return read<i8>(a, pos);}
        inline static void writeI8(const unsigned char *a, unsigned int *pos, i8 val)                  {write<i8>(a, pos, val);}

        inline static u16 readU16(const unsigned char *a, unsigned int *pos)                           {return read<u16>(a, pos);}
        inline static void writeU16(const unsigned char *a, unsigned int *pos, u16 val)                {write<u16>(a, pos, val);}
        inline static i16 readI16(const unsigned char *a, unsigned int *pos)                           {return read<i16>(a, pos);}
        inline static void writeI16(const unsigned char *a, unsigned int *pos, i16 val)                {write<i16>(a, pos, val);}

        inline static u24 readU24(const unsigned char *a, unsigned int *pos)                           {u24 value = 0; pointerCopy(0x03U, (unsigned char *)(a + (*pos)), (unsigned char *)&value, pos, CONVERT_LSB); return (value & 0x00FFFFFFU);}
        inline static void writeU24(const unsigned char *a, unsigned int *pos, u24 val)                {pointerCopy(0x03U, (unsigned char *)&val, (unsigned char *)(a + (*pos)), pos, CONVERT_LSB);}
        inline static i24 readI24(const unsigned char *a, unsigned int *pos)                           {
            i24 value = 0;
            pointerCopy(0x03U, (unsigned char *)(a + (*pos)), (unsigned char *)&value, pos, CONVERT_LSB);
            value = (value & 0x00800000U) ? (value | 0xFF000000U) : (value & 0x00FFFFFFU);
            return value;
        }
        inline static void writeI24(const unsigned char *a, unsigned int *pos, u24 val)                {pointerCopy(0x03U, (unsigned char *)&val, (unsigned char *)(a + (*pos)), pos, CONVERT_LSB);}

        inline static u32 readU32(const unsigned char *a, unsigned int *pos)                           {return read<u32>(a, pos);}
        inline static void writeU32(const unsigned char *a, unsigned int *pos, u32 val)                {write<u32>(a, pos, val);}
        inline static i32 readI32(const unsigned char *a, unsigned int *pos)                           {return read<i32>(a, pos);}
        inline static void writeI32(const unsigned char *a, unsigned int *pos, i32 val)                {write<i32>(a, pos, val);}

        inline static u64 readU64(const unsigned char *a, unsigned int *pos)                           {return read<u64>(a, pos);}
        inline static void writeU64(const unsigned char *a, unsigned int *pos, u64 val)                {write<u64>(a, pos, val);}
        inline static i64 readI64(const unsigned char *a, unsigned int *pos)                           {return read<i64>(a, pos);}
        inline static void writeI64(const unsigned char *a, unsigned int *pos, i64 val)                {write<i64>(a, pos, val);}


        inline static f32 readFloat(const unsigned char *a, unsigned int *pos)                         {return read<f32>(a, pos);}
        inline static void writeFloat(const unsigned char *a, unsigned int *pos, f32 val)              {write<f32>(a, pos, val);}

        inline static f64 readDouble(const unsigned char *a, unsigned int *pos)                        {return read<f64>(a, pos);}
        inline static void writeDouble(const unsigned char *a, unsigned int *pos, f64 val)             {write<f64>(a, pos, val);}

        inline static f128 readLongDouble(const unsigned char *a, unsigned int *pos)                   {return read<f128>(a, pos);}
        inline static void writeLongDouble(const unsigned char *a, unsigned int *pos, f128 val)        {write<f128>(a, pos, val);}
    };
};

// CONVERT_MSB <-------------------------------------------------------------
// read
template <class T>
T Convert::FB::read(const unsigned char *a, unsigned int *pos) {
    T value = 0;
    pointerCopy(sizeof(T), (unsigned char *)(a + (*pos)), (unsigned char *)&value, pos, CONVERT_MSB);
    return value;
}


template <class T>
T Convert::FB::read(const unsigned char *a, unsigned int pos) {
    T value = 0;
    pointerCopy(sizeof(T), (unsigned char *)(a + pos), (unsigned char *)&value, &pos, CONVERT_MSB);
    return value;
}

template <class T>
T Convert::FB::readCheck(const unsigned char *a, unsigned int *pos, unsigned int bufSize) {
    T value = 0;
    if((*pos + sizeof(T)) > bufSize) {
        return T();
    }
    
    pointerCopy(sizeof(T), (unsigned char *)(a + (*pos)), (unsigned char *)&value, pos, CONVERT_MSB);
    return value;
}


template <class T>
T Convert::FB::readCheck(const unsigned char *a, unsigned int pos, unsigned int bufSize) {
    T value = 0;
    if((pos + sizeof(T)) > bufSize) {
        return T();
    }

    pointerCopy(sizeof(T), (unsigned char *)(a + pos), (unsigned char *)&value, &pos, CONVERT_MSB);
    return value;
}


// write
template <class T>
void Convert::FB::write(const unsigned char *a, unsigned int *pos, T val) {
    pointerCopy(sizeof(T), (unsigned char *)&val, (unsigned char *)(a + (*pos)), pos, CONVERT_MSB);
}

template <class T>
void Convert::FB::write(const unsigned char *a, unsigned int pos, T val) {
    pointerCopy(sizeof(T), (unsigned char *)&val, (unsigned char *)(a + pos), &pos, CONVERT_MSB);
}

template <class T>
void Convert::FB::writeCheck(const unsigned char *a, unsigned int *pos, unsigned int bufSize, T val) {
    if((*pos + sizeof(T)) > bufSize) {
        return;
    }

    pointerCopy(sizeof(T), (unsigned char *)&val, (unsigned char *)(a + (*pos)), pos, CONVERT_MSB);
}

template <class T>
void Convert::FB::writeCheck(const unsigned char *a, unsigned int pos, unsigned int bufSize, T val) {
    if((pos + sizeof(T)) > bufSize) {
        return;
    }

    pointerCopy(sizeof(T), (unsigned char *)&val, (unsigned char *)(a + pos), &pos, CONVERT_MSB);
}



// CONVERT_LSB <-------------------------------------------------------------
// read
template <class T>
T Convert::FL::read(const unsigned char *a, unsigned int *pos) {
    T value = 0;
    pointerCopy(sizeof(T), (unsigned char *)(a + (*pos)), (unsigned char *)&value, pos, CONVERT_LSB);
    return value;
}

template <class T>
T Convert::FL::read(const unsigned char *a, unsigned int pos) {
    T value = 0;
    pointerCopy(sizeof(T), (unsigned char *)(a + pos), (unsigned char *)&value, &pos, CONVERT_LSB);
    return value;
}

template <class T>
T Convert::FL::readCheck(const unsigned char *a, unsigned int *pos, unsigned int bufSize) {
    T value = 0;
    if((*pos + sizeof(T)) > bufSize) {
        return T();
    }

    pointerCopy(sizeof(T), (unsigned char *)(a + (*pos)), (unsigned char *)&value, pos, CONVERT_LSB);
    return value;
}


template <class T>
T Convert::FL::readCheck(const unsigned char *a, unsigned int pos, unsigned int bufSize) {
    T value = 0;
    if((pos + sizeof(T)) > bufSize) {
        return T();
    }

    pointerCopy(sizeof(T), (unsigned char *)(a + pos), (unsigned char *)&value, &pos, CONVERT_LSB);
    return value;
}

//write
template <class T>
void Convert::FL::write(const unsigned char *a, unsigned int *pos, T val) {
    pointerCopy(sizeof(T), (unsigned char *)&val, (unsigned char *)(a + (*pos)), pos, CONVERT_LSB);
}

template <class T>
void Convert::FL::write(const unsigned char *a, unsigned int pos, T val) {
    pointerCopy(sizeof(T), (unsigned char *)&val, (unsigned char *)(a + pos), &pos, CONVERT_LSB);
}

template <class T>
void Convert::FL::writeCheck(const unsigned char *a, unsigned int *pos, unsigned int bufSize, T val) {
    if((*pos + sizeof(T)) > bufSize) {
        return;
    }

    pointerCopy(sizeof(T), (unsigned char *)&val, (unsigned char *)(a + (*pos)), pos, CONVERT_LSB);
}

template <class T>
void Convert::FL::writeCheck(const unsigned char *a, unsigned int pos, unsigned int bufSize, T val) {
    if((pos + sizeof(T)) > bufSize) {
        return;
    }

    pointerCopy(sizeof(T), (unsigned char *)&val, (unsigned char *)(a + pos), &pos, CONVERT_LSB);
}


#endif // CONVERT_H
