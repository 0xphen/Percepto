// TestHelpers.h
#pragma once

#include <gtest/gtest.h>

#define EXPECT_VEC3_EQ(v1, v2)                     \
  do {                                             \
    EXPECT_DOUBLE_EQ((v1).x, (v2).x);              \
    EXPECT_DOUBLE_EQ((v1).y, (v2).y);              \
    EXPECT_DOUBLE_EQ((v1).z, (v2).z);              \
  } while (0)
