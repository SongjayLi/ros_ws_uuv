#ifndef DATA_UNIT_H
#define DATA_UNIT_H

#include <iostream>

class DataWithTimestamp {
    private:
        float data1;
        float data2;
        float data3;
        float data4;
        uint64_t timestamp;
    
    public:
        // 构造函数，使用四个数据和原始时间戳创建新对象
        DataWithTimestamp(float d1, float d2, float d3, float d4, uint64_t ts)
            : data1(d1), data2(d2), data3(d3), data4(d4), timestamp(ts) {}
    
        // 获取第一个数据的方法
        float getData1() const {
            return data1;
        }
    
        // 获取第二个数据的方法
        float getData2() const {
            return data2;
        }
    
        // 获取第三个数据的方法
        float getData3() const {
            return data3;
        }
    
        // 获取第四个数据的方法
        float getData4() const {
            return data4;
        }
    
        // 一次获取四个数据的方法，这里使用引用传递数组
        void getAllData(float& d1, float& d2, float& d3, float& d4) const {
            d1 = data1;
            d2 = data2;
            d3 = data3;
            d4 = data4;
        }
    
        // 获取原始时间戳的方法
        uint64_t getTimestamp() const {
            return timestamp;
        }
    
        // 获取以秒为单位时间的方法
        double getTimeInSeconds() const {
            return static_cast<double>(timestamp) / 1000000.0;
        }
    };


#endif