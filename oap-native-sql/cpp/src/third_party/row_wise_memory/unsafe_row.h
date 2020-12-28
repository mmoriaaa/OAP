#pragma once

#include <arrow/util/decimal.h>
#include <assert.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include <string>

#include "third_party/row_wise_memory/native_memory.h"

#define TEMP_UNSAFEROW_BUFFER_SIZE 1024
static constexpr uint8_t kBitmask[] = {1, 2, 4, 8, 16, 32, 64, 128};

/* Unsafe Row Layout (This unsafe row only used to append all fields data as continuous
 * memory, unable to be get data from)
 *
 * | validity | col 0 | col 1 | col 2 | ...
 * explain:
 * validity: n fields = (n/8 + 1) bytes
 * col: each col has variable size
 * cursor: used to pointer to cur unused pos
 *
 */
struct UnsafeRow {
  int numFields;
  char* data = nullptr;
  std::vector<int> col_byte_list_;
  int bytes_total = 0;
  int validity_size_;
  int cursor;
  UnsafeRow() {}
  UnsafeRow(int numFields, std::vector<int> col_byte_list = {}) : numFields(numFields) {
    auto validity_size_ = (numFields / 8) + 1;
    cursor = validity_size_;
    data = (char*)nativeMalloc(TEMP_UNSAFEROW_BUFFER_SIZE, MEMTYPE_ROW);
    if (col_byte_list.size() != 0) {
      for (int i : col_byte_list) {
        bytes_total += i;
      }
    }
    col_byte_list_ = col_byte_list;
    memset(data, 0, validity_size_);
  }
  ~UnsafeRow() {
    if (data) {
      nativeFree(data);
    }
  }
  int sizeInBytes() { return cursor; }
  void reset() {
    memset(data, 0, cursor);
    auto validity_size = (numFields / 8) + 1;
    cursor = validity_size;
  }
  bool isNullExists() {
    for (int i = 0; i < ((numFields / 8) + 1); i++) {
      if (data[i] != 0) return true;
    }
    return false;
  }
  auto getData(int id, int64_t col_id) {
    int bytes_before = col_id > 0 ? col_byte_list_[col_id - 1] : 0;
    // *((T*)(row->data + row->cursor))
    auto val = *((uint32_t*)(data + 1 + id * bytes_total + bytes_before));
    // return *((uint32_t*)(data + 1 + id * bytes_total + bytes_before));
    return *(data + 1 + id * bytes_total + bytes_before);
  }
};

static inline int calculateBitSetWidthInBytes(int numFields) {
  return ((numFields / 8) + 1);
}

static inline int getSizeInBytes(UnsafeRow* row) { return row->cursor; }

static inline int roundNumberOfBytesToNearestWord(int numBytes) {
  int remainder = numBytes & 0x07;  // This is equivalent to `numBytes % 8`
  if (remainder == 0) {
    return numBytes;
  } else {
    return numBytes + (8 - remainder);
  }
}

static inline void zeroOutPaddingBytes(UnsafeRow* row, int numBytes) {
  if ((numBytes & 0x07) > 0) {
    *((int64_t*)(char*)(row->data + row->cursor + ((numBytes >> 3) << 3))) = 0L;
  }
}

static inline void setNullAt(UnsafeRow* row, int index) {
  assert((index >= 0) && (index < row->numFields));
  auto bitSetIdx = index >> 3;  // mod 8
  *(row->data + bitSetIdx) |= kBitmask[index % 8];
}

template <typename T>
using is_number_alike =
    std::integral_constant<bool, std::is_arithmetic<T>::value ||
                                     std::is_floating_point<T>::value>;

template <typename T, typename std::enable_if_t<is_number_alike<T>::value>* = nullptr>
static inline void appendToUnsafeRow(UnsafeRow* row, const int& index, const T& val) {
  *((T*)(row->data + row->cursor)) = val;
  row->cursor += sizeof(T);
  // std::cout << "cursor is: " << row->cursor << ", val is: " << val << std::endl;
}

static inline void appendToUnsafeRow(UnsafeRow* row, const int& index,
                                     const std::string& str) {
  int numBytes = str.size();
  // int roundedSize = roundNumberOfBytesToNearestWord(numBytes);

  // zeroOutPaddingBytes(row, numBytes);
  memcpy(row->data + row->cursor, str.c_str(), numBytes);

  // move the cursor forward.
  row->cursor += numBytes;
}

static inline void appendToUnsafeRow(UnsafeRow* row, const int& index,
                                     const arrow::Decimal128& dcm) {
  int numBytes = 16;
  zeroOutPaddingBytes(row, numBytes);
  memcpy(row->data + row->cursor, dcm.ToBytes().data(), numBytes);
  // move the cursor forward.
  row->cursor += numBytes;
}
