/**
*    @author : koseng (Lintang)
*    @brief : Circular buffer
*/
#pragma once

namespace micron{
  
template <typename T>
class CircularBuffer{
public:
  CircularBuffer(size_t _size)
    : data_(new T[_size])
    , max_size_(_size)
    , current_idx_(0)
    , oldest_idx_(0)
    , available_data_(0){
      
    }

    void push(T t){
      data_[current_idx_] = t;
      if(available_data_ == max_size_){
        oldest_idx_ = (oldest_idx_ + 1)%max_size_;
      }else{
        ++available_data_;
      }
      current_idx_ = (current_idx_ + 1)%max_size_;
    }

    T at(int _idx){
      if(available_data_ == 0)
        return T();
      return data_[(_idx + oldest_idx_)%max_size_];
    }

    void popOldest(){
      if(available_data_ == 0)return;

      --available_data_;
      data_[oldest_idx_] = T();
      oldest_idx_ = (oldest_idx_ + 1)%max_size_;           
    }

    size_t currentSize() const{
      return available_data_;
    }

    size_t capacity() const{
      return max_size_;
    }
    
private:
  T* data_;
  size_t max_size_;
  size_t current_idx_;
  size_t oldest_idx_;
  size_t available_data_;
};

} // namespace micron
