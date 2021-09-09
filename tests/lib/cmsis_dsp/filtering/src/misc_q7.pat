static const q7_t in_com1[128] = {
    0xF3, 0xAE, 0x42, 0x21, 0x19, 0xE2, 0x32, 0x15,
    0xF9, 0xC4, 0xB6, 0xE3, 0xE1, 0x49, 0x2F, 0x1A,
    0xF9, 0xE0, 0x28, 0xEA, 0xF1, 0x41, 0x7F, 0x32,
    0xD5, 0x04, 0xBF, 0x0B, 0xD0, 0xBC, 0x16, 0x20,
    0xBD, 0x08, 0xD8, 0xF4, 0x2E, 0x13, 0xFB, 0xC4,
    0x26, 0xF2, 0x05, 0x0E, 0xA9, 0x09, 0xDE, 0x42,
    0x30, 0xFC, 0x16, 0xDB, 0x17, 0xD8, 0x02, 0x2C,
    0xFD, 0x05, 0xEF, 0x02, 0x13, 0xDA, 0x03, 0x2D,
    0x24, 0x0D, 0x0D, 0xE8, 0xF4, 0xB5, 0xF6, 0xB6,
    0x1C, 0xDE, 0x09, 0x03, 0xF0, 0xCD, 0x0B, 0xB0,
    0x1D, 0x1C, 0x09, 0xBE, 0x0C, 0xA5, 0x15, 0x28,
    0xF3, 0x2A, 0x18, 0x03, 0x00, 0xC9, 0x1E, 0xC5,
    0xF5, 0x37, 0x2A, 0x93, 0x24, 0xF8, 0xDE, 0xEE,
    0xEA, 0xF2, 0x21, 0x5A, 0xD0, 0x01, 0x0C, 0xD5,
    0x39, 0x41, 0x31, 0x06, 0xD0, 0x46, 0xA0, 0x21,
    0xDB, 0x0F, 0x27, 0x4A, 0x04, 0xDC, 0xE7, 0xE9
    };

static const q7_t in_com2[128] = {
    0xA2, 0x8D, 0xDE, 0x29, 0xD9, 0xBC, 0xC4, 0xF1,
    0x42, 0x80, 0x08, 0xF9, 0xB5, 0xA2, 0xC7, 0x1C,
    0x15, 0x24, 0x19, 0xF5, 0x16, 0x35, 0xF1, 0x09,
    0xFE, 0x01, 0x12, 0x00, 0xEF, 0xC6, 0xAC, 0xDB,
    0x29, 0xC6, 0xF0, 0x08, 0xF7, 0x17, 0xEB, 0x2B,
    0x4B, 0x10, 0xF7, 0x1F, 0x00, 0xBC, 0x00, 0xEC,
    0xBD, 0xCE, 0x29, 0xF4, 0x46, 0x57, 0xEB, 0x27,
    0xED, 0xDE, 0xEE, 0x52, 0xE7, 0x26, 0x93, 0xBC,
    0x2D, 0x07, 0xF4, 0x3D, 0xC2, 0xE2, 0xDC, 0x19,
    0xE9, 0xE2, 0x27, 0x0B, 0x3C, 0x09, 0xDC, 0xD3,
    0x14, 0xEA, 0xBF, 0x32, 0xFC, 0xAB, 0x1B, 0xE3,
    0x06, 0xC0, 0xD7, 0xFF, 0x07, 0x34, 0x24, 0xEC,
    0x29, 0xE1, 0x1B, 0x4B, 0x2C, 0xF0, 0xEE, 0x2F,
    0xE6, 0xE3, 0xE2, 0x14, 0x35, 0x0D, 0x1B, 0xA6,
    0xD5, 0x35, 0x05, 0x52, 0xBB, 0x65, 0xF4, 0x17,
    0x04, 0xD9, 0x1A, 0x17, 0x09, 0x3C, 0x34, 0x5A
    };

static const q7_t in_partial1[128] = {
    0x21, 0xFB, 0xD3, 0x45, 0x01, 0x65, 0xEE, 0xE3,
    0xF4, 0x12, 0xF5, 0xFB, 0x3D, 0x2B, 0xF6, 0xC5,
    0xF8, 0x83, 0xE2, 0x07, 0x38, 0x2B, 0x08, 0x18,
    0xF0, 0x31, 0x19, 0x12, 0x0C, 0xE4, 0xE2, 0x10,
    0x02, 0x28, 0x03, 0x05, 0x0D, 0x20, 0x1A, 0x31,
    0x22, 0xC8, 0xDC, 0xE0, 0x43, 0xDF, 0xEB, 0xFB,
    0x1F, 0x28, 0x28, 0xB1, 0xDA, 0xE7, 0xD8, 0xEB,
    0x37, 0xEF, 0xF7, 0xE2, 0x43, 0xCB, 0xA7, 0xDF,
    0xE5, 0xD1, 0xFF, 0x20, 0x03, 0x0B, 0xFF, 0xEA,
    0x57, 0xD8, 0xFC, 0x50, 0x0B, 0xD7, 0xC4, 0x05,
    0xE3, 0xD1, 0xAE, 0x1A, 0x87, 0x1B, 0x23, 0xF5,
    0xF9, 0x1C, 0x15, 0xF0, 0xDF, 0xFB, 0x18, 0xF6,
    0xC9, 0x20, 0xA3, 0x18, 0x1F, 0xFA, 0x39, 0x17,
    0xD7, 0x0E, 0x0B, 0x29, 0xFC, 0xDB, 0x21, 0x05,
    0xE2, 0x4F, 0xFF, 0xD4, 0x01, 0xCF, 0x3C, 0x1A,
    0x39, 0xF2, 0x24, 0x80, 0xF2, 0x3B, 0x15, 0x17
    };

static const q7_t in_partial2[128] = {
    0xF1, 0xD9, 0x12, 0x29, 0x07, 0xDD, 0xEB, 0x1E,
    0x69, 0x24, 0xCA, 0x97, 0x41, 0x1E, 0xBD, 0xE5,
    0x7F, 0xAC, 0x27, 0xE1, 0xE3, 0xE6, 0xDC, 0x19,
    0x0F, 0x49, 0xC5, 0xFF, 0xFB, 0xE8, 0x8D, 0xF8,
    0x0F, 0xE1, 0x10, 0xB7, 0x2A, 0xEE, 0x3E, 0x0D,
    0xDC, 0x48, 0xFF, 0xF9, 0x30, 0xE8, 0xEC, 0xC1,
    0x14, 0x50, 0x04, 0xAE, 0x15, 0x0C, 0xEB, 0xB9,
    0x49, 0x50, 0x33, 0x1B, 0x09, 0x4C, 0x28, 0x55,
    0xE9, 0xE4, 0x09, 0x36, 0xA8, 0x04, 0x02, 0x48,
    0xFE, 0x0E, 0x03, 0xD7, 0xE2, 0x05, 0xAC, 0x1B,
    0xED, 0xCA, 0xD1, 0x04, 0xF6, 0xD3, 0xD9, 0x1C,
    0x02, 0x28, 0xFE, 0xA9, 0xB3, 0x13, 0xC9, 0x03,
    0x06, 0xE5, 0xE1, 0xC1, 0x4E, 0x41, 0xCA, 0xBD,
    0xE0, 0x0F, 0x2D, 0x16, 0xE4, 0x01, 0x30, 0xD4,
    0xE7, 0x01, 0xE1, 0x90, 0x45, 0x10, 0xD8, 0x15,
    0x01, 0x0F, 0xC4, 0x20, 0xCF, 0x05, 0xE4, 0xD0
    };

static const q7_t ref_correlate_30_31[61] = {
    0x08, 0x3C, 0xFC, 0xD8, 0xD6, 0xF8, 0xF3, 0xE5,
    0xF2, 0x2C, 0x29, 0x4C, 0x42, 0xD7, 0x9E, 0xDF,
    0x0D, 0x61, 0x1B, 0xE0, 0xC1, 0xB6, 0xD1, 0x80,
    0xC0, 0x35, 0x7F, 0x48, 0x51, 0x7A, 0x65, 0xFB,
    0x80, 0x0C, 0x20, 0x3F, 0x06, 0x00, 0xA0, 0x80,
    0x1B, 0x0D, 0xAA, 0x80, 0xEF, 0x6E, 0xDE, 0xF8,
    0xB6, 0x74, 0x12, 0x80, 0xAF, 0x22, 0x63, 0x3A,
    0x1D, 0x35, 0x60, 0x32, 0x00
    };

static const q7_t ref_correlate_30_32[63] = {
    0x04, 0x20, 0x28, 0xF2, 0xD0, 0xDF, 0xEA, 0xED,
    0xE7, 0x04, 0x42, 0x31, 0x55, 0x2D, 0xC9, 0x96,
    0xE1, 0x17, 0x56, 0x22, 0xE4, 0xAE, 0x90, 0xC2,
    0x80, 0xBF, 0x48, 0x7F, 0x56, 0x64, 0x7A, 0x65,
    0xFB, 0x80, 0x0C, 0x20, 0x3F, 0x06, 0x00, 0xA0,
    0x80, 0x1B, 0x0D, 0xAA, 0x80, 0xEF, 0x6E, 0xDE,
    0xF8, 0xB6, 0x74, 0x12, 0x80, 0xAF, 0x22, 0x63,
    0x3A, 0x1D, 0x35, 0x60, 0x32, 0x00, 0x00
    };

static const q7_t ref_correlate_30_33[65] = {
    0xFC, 0xE9, 0x35, 0x33, 0xFA, 0xC7, 0xEF, 0xF1,
    0xEB, 0xD4, 0xEC, 0x38, 0x28, 0x6C, 0x3C, 0xD2,
    0x94, 0xD7, 0x23, 0x4F, 0x1D, 0xF9, 0xD7, 0xA0,
    0xB4, 0x80, 0xAA, 0x4C, 0x7F, 0x40, 0x64, 0x7A,
    0x65, 0xFB, 0x80, 0x0C, 0x20, 0x3F, 0x06, 0x00,
    0xA0, 0x80, 0x1B, 0x0D, 0xAA, 0x80, 0xEF, 0x6E,
    0xDE, 0xF8, 0xB6, 0x74, 0x12, 0x80, 0xAF, 0x22,
    0x63, 0x3A, 0x1D, 0x35, 0x60, 0x32, 0x00, 0x00,
    0x00
    };

static const q7_t ref_correlate_30_34[67] = {
    0x06, 0x21, 0xCB, 0x26, 0x27, 0x08, 0xB0, 0xE5,
    0xF4, 0x06, 0xF6, 0xF9, 0x46, 0x07, 0x57, 0x30,
    0xD5, 0xA3, 0xC5, 0x2D, 0x55, 0xFF, 0xBF, 0xC1,
    0xB4, 0xB2, 0x80, 0xA5, 0x61, 0x7F, 0x40, 0x64,
    0x7A, 0x65, 0xFB, 0x80, 0x0C, 0x20, 0x3F, 0x06,
    0x00, 0xA0, 0x80, 0x1B, 0x0D, 0xAA, 0x80, 0xEF,
    0x6E, 0xDE, 0xF8, 0xB6, 0x74, 0x12, 0x80, 0xAF,
    0x22, 0x63, 0x3A, 0x1D, 0x35, 0x60, 0x32, 0x00,
    0x00, 0x00, 0x00
    };

static const q7_t ref_correlate_30_49[97] = {
    0x07, 0x2D, 0xEB, 0xEB, 0x19, 0xE6, 0xC6, 0xF4,
    0x02, 0xDD, 0x2D, 0x65, 0x33, 0x0B, 0xED, 0x25,
    0xDF, 0x95, 0xD6, 0x17, 0x11, 0x96, 0xD6, 0xEB,
    0xF9, 0xA6, 0x27, 0x77, 0x06, 0x7F, 0x7F, 0x1B,
    0xB7, 0xB1, 0x0A, 0x39, 0xE0, 0x80, 0xBE, 0xB1,
    0xB2, 0x80, 0xA7, 0x6A, 0x7F, 0x40, 0x64, 0x7A,
    0x65, 0xFB, 0x80, 0x0C, 0x20, 0x3F, 0x06, 0x00,
    0xA0, 0x80, 0x1B, 0x0D, 0xAA, 0x80, 0xEF, 0x6E,
    0xDE, 0xF8, 0xB6, 0x74, 0x12, 0x80, 0xAF, 0x22,
    0x63, 0x3A, 0x1D, 0x35, 0x60, 0x32, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00
    };

static const q7_t ref_correlate_31_31[61] = {
    0x08, 0x3C, 0xFC, 0xD8, 0xD6, 0xF8, 0xF3, 0xE5,
    0xF2, 0x2C, 0x29, 0x4C, 0x42, 0xD7, 0x9E, 0xDF,
    0x0D, 0x61, 0x1B, 0xE0, 0xC1, 0xB6, 0xD1, 0x80,
    0xC0, 0x35, 0x7F, 0x48, 0x51, 0x7A, 0x56, 0xF0,
    0x80, 0x0C, 0x23, 0x40, 0x05, 0x01, 0x9D, 0x80,
    0x1F, 0x0C, 0xAF, 0x80, 0xF3, 0x73, 0xD4, 0xE8,
    0xA9, 0x72, 0x13, 0x80, 0xBB, 0x20, 0x58, 0x2E,
    0x17, 0x3C, 0x5A, 0x1E, 0xF0
    };

static const q7_t ref_correlate_31_32[63] = {
    0x04, 0x20, 0x28, 0xF2, 0xD0, 0xDF, 0xEA, 0xED,
    0xE7, 0x04, 0x42, 0x31, 0x55, 0x2D, 0xC9, 0x96,
    0xE1, 0x17, 0x56, 0x22, 0xE4, 0xAE, 0x90, 0xC2,
    0x80, 0xBF, 0x48, 0x7F, 0x56, 0x64, 0x74, 0x56,
    0xF0, 0x80, 0x0C, 0x23, 0x40, 0x05, 0x01, 0x9D,
    0x80, 0x1F, 0x0C, 0xAF, 0x80, 0xF3, 0x73, 0xD4,
    0xE8, 0xA9, 0x72, 0x13, 0x80, 0xBB, 0x20, 0x58,
    0x2E, 0x17, 0x3C, 0x5A, 0x1E, 0xF0, 0x00
    };

static const q7_t ref_correlate_31_33[65] = {
    0xFC, 0xE9, 0x35, 0x33, 0xFA, 0xC7, 0xEF, 0xF1,
    0xEB, 0xD4, 0xEC, 0x38, 0x28, 0x6C, 0x3C, 0xD2,
    0x94, 0xD7, 0x23, 0x4F, 0x1D, 0xF9, 0xD7, 0xA0,
    0xB4, 0x80, 0xAA, 0x4C, 0x7F, 0x40, 0x6B, 0x74,
    0x56, 0xF0, 0x80, 0x0C, 0x23, 0x40, 0x05, 0x01,
    0x9D, 0x80, 0x1F, 0x0C, 0xAF, 0x80, 0xF3, 0x73,
    0xD4, 0xE8, 0xA9, 0x72, 0x13, 0x80, 0xBB, 0x20,
    0x58, 0x2E, 0x17, 0x3C, 0x5A, 0x1E, 0xF0, 0x00,
    0x00
    };

static const q7_t ref_correlate_31_34[67] = {
    0x06, 0x21, 0xCB, 0x26, 0x27, 0x08, 0xB0, 0xE5,
    0xF4, 0x06, 0xF6, 0xF9, 0x46, 0x07, 0x57, 0x30,
    0xD5, 0xA3, 0xC5, 0x2D, 0x55, 0xFF, 0xBF, 0xC1,
    0xB4, 0xB2, 0x80, 0xA5, 0x61, 0x7F, 0x36, 0x6B,
    0x74, 0x56, 0xF0, 0x80, 0x0C, 0x23, 0x40, 0x05,
    0x01, 0x9D, 0x80, 0x1F, 0x0C, 0xAF, 0x80, 0xF3,
    0x73, 0xD4, 0xE8, 0xA9, 0x72, 0x13, 0x80, 0xBB,
    0x20, 0x58, 0x2E, 0x17, 0x3C, 0x5A, 0x1E, 0xF0,
    0x00, 0x00, 0x00
    };

static const q7_t ref_correlate_31_49[97] = {
    0x07, 0x2D, 0xEB, 0xEB, 0x19, 0xE6, 0xC6, 0xF4,
    0x02, 0xDD, 0x2D, 0x65, 0x33, 0x0B, 0xED, 0x25,
    0xDF, 0x95, 0xD6, 0x17, 0x11, 0x96, 0xD6, 0xEB,
    0xF9, 0xA6, 0x27, 0x77, 0x06, 0x7F, 0x7F, 0x18,
    0xB7, 0xA5, 0x0A, 0x3E, 0xDF, 0x80, 0xCB, 0xB9,
    0xAE, 0x80, 0xA5, 0x6B, 0x7F, 0x36, 0x6B, 0x74,
    0x56, 0xF0, 0x80, 0x0C, 0x23, 0x40, 0x05, 0x01,
    0x9D, 0x80, 0x1F, 0x0C, 0xAF, 0x80, 0xF3, 0x73,
    0xD4, 0xE8, 0xA9, 0x72, 0x13, 0x80, 0xBB, 0x20,
    0x58, 0x2E, 0x17, 0x3C, 0x5A, 0x1E, 0xF0, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00
    };

static const q7_t ref_correlate_32_31[63] = {
    0x00, 0x08, 0x3C, 0xFC, 0xD8, 0xD6, 0xF8, 0xF3,
    0xE5, 0xF2, 0x2C, 0x29, 0x4C, 0x42, 0xD7, 0x9E,
    0xDF, 0x0D, 0x61, 0x1B, 0xE0, 0xC1, 0xB6, 0xD1,
    0x80, 0xC0, 0x35, 0x7F, 0x48, 0x51, 0x7A, 0x56,
    0xDC, 0x80, 0x07, 0x24, 0x44, 0x05, 0x01, 0x9F,
    0x80, 0x2C, 0x11, 0xAC, 0x80, 0xFC, 0x78, 0xDB,
    0xDA, 0x91, 0x60, 0x11, 0x80, 0x9B, 0x30, 0x55,
    0x1F, 0x06, 0x32, 0x64, 0x15, 0xD3, 0xE9
    };

static const q7_t ref_correlate_32_32[63] = {
    0x04, 0x20, 0x28, 0xF2, 0xD0, 0xDF, 0xEA, 0xED,
    0xE7, 0x04, 0x42, 0x31, 0x55, 0x2D, 0xC9, 0x96,
    0xE1, 0x17, 0x56, 0x22, 0xE4, 0xAE, 0x90, 0xC2,
    0x80, 0xBF, 0x48, 0x7F, 0x56, 0x64, 0x74, 0x4D,
    0xDC, 0x80, 0x07, 0x24, 0x44, 0x05, 0x01, 0x9F,
    0x80, 0x2C, 0x11, 0xAC, 0x80, 0xFC, 0x78, 0xDB,
    0xDA, 0x91, 0x60, 0x11, 0x80, 0x9B, 0x30, 0x55,
    0x1F, 0x06, 0x32, 0x64, 0x15, 0xD3, 0xE9
    };

static const q7_t ref_correlate_32_33[65] = {
    0xFC, 0xE9, 0x35, 0x33, 0xFA, 0xC7, 0xEF, 0xF1,
    0xEB, 0xD4, 0xEC, 0x38, 0x28, 0x6C, 0x3C, 0xD2,
    0x94, 0xD7, 0x23, 0x4F, 0x1D, 0xF9, 0xD7, 0xA0,
    0xB4, 0x80, 0xAA, 0x4C, 0x7F, 0x40, 0x6B, 0x7E,
    0x4D, 0xDC, 0x80, 0x07, 0x24, 0x44, 0x05, 0x01,
    0x9F, 0x80, 0x2C, 0x11, 0xAC, 0x80, 0xFC, 0x78,
    0xDB, 0xDA, 0x91, 0x60, 0x11, 0x80, 0x9B, 0x30,
    0x55, 0x1F, 0x06, 0x32, 0x64, 0x15, 0xD3, 0xE9,
    0x00
    };

static const q7_t ref_correlate_32_34[67] = {
    0x06, 0x21, 0xCB, 0x26, 0x27, 0x08, 0xB0, 0xE5,
    0xF4, 0x06, 0xF6, 0xF9, 0x46, 0x07, 0x57, 0x30,
    0xD5, 0xA3, 0xC5, 0x2D, 0x55, 0xFF, 0xBF, 0xC1,
    0xB4, 0xB2, 0x80, 0xA5, 0x61, 0x7F, 0x36, 0x5D,
    0x7E, 0x4D, 0xDC, 0x80, 0x07, 0x24, 0x44, 0x05,
    0x01, 0x9F, 0x80, 0x2C, 0x11, 0xAC, 0x80, 0xFC,
    0x78, 0xDB, 0xDA, 0x91, 0x60, 0x11, 0x80, 0x9B,
    0x30, 0x55, 0x1F, 0x06, 0x32, 0x64, 0x15, 0xD3,
    0xE9, 0x00, 0x00
    };

static const q7_t ref_correlate_32_49[97] = {
    0x07, 0x2D, 0xEB, 0xEB, 0x19, 0xE6, 0xC6, 0xF4,
    0x02, 0xDD, 0x2D, 0x65, 0x33, 0x0B, 0xED, 0x25,
    0xDF, 0x95, 0xD6, 0x17, 0x11, 0x96, 0xD6, 0xEB,
    0xF9, 0xA6, 0x27, 0x77, 0x06, 0x7F, 0x7F, 0x07,
    0xB2, 0xA5, 0xF9, 0x3E, 0xE6, 0x80, 0xCF, 0xCC,
    0xB9, 0x80, 0xAB, 0x69, 0x7F, 0x32, 0x5D, 0x7E,
    0x4D, 0xDC, 0x80, 0x07, 0x24, 0x44, 0x05, 0x01,
    0x9F, 0x80, 0x2C, 0x11, 0xAC, 0x80, 0xFC, 0x78,
    0xDB, 0xDA, 0x91, 0x60, 0x11, 0x80, 0x9B, 0x30,
    0x55, 0x1F, 0x06, 0x32, 0x64, 0x15, 0xD3, 0xE9,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00
    };

static const q7_t ref_correlate_33_31[65] = {
    0x00, 0x00, 0x08, 0x3C, 0xFC, 0xD8, 0xD6, 0xF8,
    0xF3, 0xE5, 0xF2, 0x2C, 0x29, 0x4C, 0x42, 0xD7,
    0x9E, 0xDF, 0x0D, 0x61, 0x1B, 0xE0, 0xC1, 0xB6,
    0xD1, 0x80, 0xC0, 0x35, 0x7F, 0x48, 0x51, 0x7A,
    0x56, 0xDC, 0x8D, 0x26, 0x2C, 0x44, 0xFC, 0x01,
    0xA0, 0x80, 0x34, 0xF6, 0xA0, 0x80, 0xEF, 0x65,
    0xD0, 0xCB, 0xAF, 0x7F, 0x38, 0x80, 0x97, 0x73,
    0x32, 0x27, 0x25, 0x56, 0x78, 0x00, 0xE5, 0x25,
    0x31
    };

static const q7_t ref_correlate_33_32[65] = {
    0x00, 0x04, 0x20, 0x28, 0xF2, 0xD0, 0xDF, 0xEA,
    0xED, 0xE7, 0x04, 0x42, 0x31, 0x55, 0x2D, 0xC9,
    0x96, 0xE1, 0x17, 0x56, 0x22, 0xE4, 0xAE, 0x90,
    0xC2, 0x80, 0xBF, 0x48, 0x7F, 0x56, 0x64, 0x74,
    0x4D, 0xEF, 0x8D, 0x26, 0x2C, 0x44, 0xFC, 0x01,
    0xA0, 0x80, 0x34, 0xF6, 0xA0, 0x80, 0xEF, 0x65,
    0xD0, 0xCB, 0xAF, 0x7F, 0x38, 0x80, 0x97, 0x73,
    0x32, 0x27, 0x25, 0x56, 0x78, 0x00, 0xE5, 0x25,
    0x31
    };

static const q7_t ref_correlate_33_33[65] = {
    0xFC, 0xE9, 0x35, 0x33, 0xFA, 0xC7, 0xEF, 0xF1,
    0xEB, 0xD4, 0xEC, 0x38, 0x28, 0x6C, 0x3C, 0xD2,
    0x94, 0xD7, 0x23, 0x4F, 0x1D, 0xF9, 0xD7, 0xA0,
    0xB4, 0x80, 0xAA, 0x4C, 0x7F, 0x40, 0x6B, 0x7E,
    0x38, 0xEF, 0x8D, 0x26, 0x2C, 0x44, 0xFC, 0x01,
    0xA0, 0x80, 0x34, 0xF6, 0xA0, 0x80, 0xEF, 0x65,
    0xD0, 0xCB, 0xAF, 0x7F, 0x38, 0x80, 0x97, 0x73,
    0x32, 0x27, 0x25, 0x56, 0x78, 0x00, 0xE5, 0x25,
    0x31
    };

static const q7_t ref_correlate_33_34[67] = {
    0x06, 0x21, 0xCB, 0x26, 0x27, 0x08, 0xB0, 0xE5,
    0xF4, 0x06, 0xF6, 0xF9, 0x46, 0x07, 0x57, 0x30,
    0xD5, 0xA3, 0xC5, 0x2D, 0x55, 0xFF, 0xBF, 0xC1,
    0xB4, 0xB2, 0x80, 0xA5, 0x61, 0x7F, 0x36, 0x5D,
    0x7F, 0x38, 0xEF, 0x8D, 0x26, 0x2C, 0x44, 0xFC,
    0x01, 0xA0, 0x80, 0x34, 0xF6, 0xA0, 0x80, 0xEF,
    0x65, 0xD0, 0xCB, 0xAF, 0x7F, 0x38, 0x80, 0x97,
    0x73, 0x32, 0x27, 0x25, 0x56, 0x78, 0x00, 0xE5,
    0x25, 0x31, 0x00
    };

static const q7_t ref_correlate_33_49[97] = {
    0x07, 0x2D, 0xEB, 0xEB, 0x19, 0xE6, 0xC6, 0xF4,
    0x02, 0xDD, 0x2D, 0x65, 0x33, 0x0B, 0xED, 0x25,
    0xDF, 0x95, 0xD6, 0x17, 0x11, 0x96, 0xD6, 0xEB,
    0xF9, 0xA6, 0x27, 0x77, 0x06, 0x7F, 0x7F, 0x07,
    0xD5, 0xAF, 0xF9, 0x62, 0xE6, 0x80, 0xD4, 0xC3,
    0x92, 0x80, 0xB6, 0x5D, 0x7F, 0x2E, 0x65, 0x7F,
    0x38, 0xEF, 0x8D, 0x26, 0x2C, 0x44, 0xFC, 0x01,
    0xA0, 0x80, 0x34, 0xF6, 0xA0, 0x80, 0xEF, 0x65,
    0xD0, 0xCB, 0xAF, 0x7F, 0x38, 0x80, 0x97, 0x73,
    0x32, 0x27, 0x25, 0x56, 0x78, 0x00, 0xE5, 0x25,
    0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00
    };

static const q7_t ref_correlate_48_31[95] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x08, 0x3C, 0xFC, 0xD8, 0xD6, 0xF8, 0xF3,
    0xE5, 0xF2, 0x2C, 0x29, 0x4C, 0x42, 0xD7, 0x9E,
    0xDF, 0x0D, 0x61, 0x1B, 0xE0, 0xC1, 0xB6, 0xD1,
    0x80, 0xC0, 0x35, 0x7F, 0x48, 0x51, 0x7A, 0x56,
    0xDC, 0x8D, 0x21, 0x43, 0x5D, 0xE8, 0xE2, 0x8F,
    0x80, 0x3F, 0xF5, 0xA4, 0x80, 0x15, 0x7F, 0xF5,
    0xAB, 0x80, 0x7F, 0x52, 0x80, 0xAF, 0x28, 0xF4,
    0x61, 0x40, 0x2C, 0x6D, 0x0C, 0x46, 0xEA, 0x7F,
    0x35, 0x05, 0x17, 0x80, 0x43, 0xF7, 0x5B, 0x12,
    0xD6, 0xF1, 0x22, 0x56, 0x07, 0xDE, 0xCF
    };

static const q7_t ref_correlate_48_32[95] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x04, 0x20, 0x28, 0xF2, 0xD0, 0xDF, 0xEA, 0xED,
    0xE7, 0x04, 0x42, 0x31, 0x55, 0x2D, 0xC9, 0x96,
    0xE1, 0x17, 0x56, 0x22, 0xE4, 0xAE, 0x90, 0xC2,
    0x80, 0xBF, 0x48, 0x7F, 0x56, 0x64, 0x74, 0x4D,
    0xEF, 0x8B, 0x2C, 0x47, 0x4F, 0xE3, 0xE3, 0xA1,
    0x80, 0x43, 0xF3, 0xA0, 0x80, 0x12, 0x7F, 0xE2,
    0xAB, 0x80, 0x7F, 0x52, 0x80, 0xAF, 0x28, 0xF4,
    0x61, 0x40, 0x2C, 0x6D, 0x0C, 0x46, 0xEA, 0x7F,
    0x35, 0x05, 0x17, 0x80, 0x43, 0xF7, 0x5B, 0x12,
    0xD6, 0xF1, 0x22, 0x56, 0x07, 0xDE, 0xCF
    };

static const q7_t ref_correlate_48_33[95] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
    0xE9, 0x35, 0x33, 0xFA, 0xC7, 0xEF, 0xF1, 0xEB,
    0xD4, 0xEC, 0x38, 0x28, 0x6C, 0x3C, 0xD2, 0x94,
    0xD7, 0x23, 0x4F, 0x1D, 0xF9, 0xD7, 0xA0, 0xB4,
    0x80, 0xAA, 0x4C, 0x7F, 0x40, 0x6B, 0x7E, 0x38,
    0xF1, 0x80, 0x29, 0x55, 0x55, 0xE1, 0xD0, 0xAD,
    0x80, 0x44, 0xF8, 0x84, 0x80, 0x08, 0x7F, 0xE2,
    0xAB, 0x80, 0x7F, 0x52, 0x80, 0xAF, 0x28, 0xF4,
    0x61, 0x40, 0x2C, 0x6D, 0x0C, 0x46, 0xEA, 0x7F,
    0x35, 0x05, 0x17, 0x80, 0x43, 0xF7, 0x5B, 0x12,
    0xD6, 0xF1, 0x22, 0x56, 0x07, 0xDE, 0xCF
    };

static const q7_t ref_correlate_48_34[95] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x21,
    0xCB, 0x26, 0x27, 0x08, 0xB0, 0xE5, 0xF4, 0x06,
    0xF6, 0xF9, 0x46, 0x07, 0x57, 0x30, 0xD5, 0xA3,
    0xC5, 0x2D, 0x55, 0xFF, 0xBF, 0xC1, 0xB4, 0xB2,
    0x80, 0xA5, 0x61, 0x7F, 0x36, 0x5D, 0x7F, 0x34,
    0x04, 0x84, 0x13, 0x4D, 0x58, 0xFC, 0xBF, 0xB3,
    0x80, 0x3E, 0x20, 0x80, 0x80, 0xE9, 0x7F, 0xE2,
    0xAB, 0x80, 0x7F, 0x52, 0x80, 0xAF, 0x28, 0xF4,
    0x61, 0x40, 0x2C, 0x6D, 0x0C, 0x46, 0xEA, 0x7F,
    0x35, 0x05, 0x17, 0x80, 0x43, 0xF7, 0x5B, 0x12,
    0xD6, 0xF1, 0x22, 0x56, 0x07, 0xDE, 0xCF
    };

static const q7_t ref_correlate_48_49[97] = {
    0x07, 0x2D, 0xEB, 0xEB, 0x19, 0xE6, 0xC6, 0xF4,
    0x02, 0xDD, 0x2D, 0x65, 0x33, 0x0B, 0xED, 0x25,
    0xDF, 0x95, 0xD6, 0x17, 0x11, 0x96, 0xD6, 0xEB,
    0xF9, 0xA6, 0x27, 0x77, 0x06, 0x7F, 0x7F, 0x07,
    0xD5, 0xAB, 0x0D, 0x6E, 0xCC, 0x80, 0xDC, 0xC0,
    0x80, 0x80, 0xC0, 0x2E, 0x7F, 0x57, 0x5D, 0x7F,
    0x2E, 0x11, 0x80, 0x14, 0x29, 0x40, 0xFE, 0xCA,
    0xD7, 0x80, 0x58, 0x18, 0x88, 0x80, 0xE9, 0x7F,
    0xE2, 0xAB, 0x80, 0x7F, 0x52, 0x80, 0xAF, 0x28,
    0xF4, 0x61, 0x40, 0x2C, 0x6D, 0x0C, 0x46, 0xEA,
    0x7F, 0x35, 0x05, 0x17, 0x80, 0x43, 0xF7, 0x5B,
    0x12, 0xD6, 0xF1, 0x22, 0x56, 0x07, 0xDE, 0xCF,
    0x00
    };

static const q7_t ref_conv_30_31[60] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xD5,
    0x94, 0x31, 0xAD, 0xD4, 0x81, 0x22, 0x7F, 0x7F,
    0x7F, 0x51, 0x50, 0xF0, 0xBD, 0xD3, 0xEB, 0xE8,
    0x0F, 0xEE, 0xC1, 0x9A, 0x8F, 0xF5, 0x1C, 0x10,
    0x2C, 0x18, 0x3E, 0x2C
    };

static const q7_t ref_conv_30_32[61] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xD8,
    0xAC, 0x1E, 0xA3, 0xCD, 0x8A, 0x13, 0x7F, 0x7F,
    0x7F, 0x67, 0x58, 0xF9, 0xA8, 0xC5, 0xE4, 0xEB,
    0x18, 0xE2, 0xC7, 0x9E, 0x80, 0xD0, 0x0E, 0x1C,
    0x2B, 0x2B, 0x3B, 0x3A, 0x14
    };

static const q7_t ref_conv_30_33[62] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xD8,
    0xA8, 0x03, 0xB8, 0xD7, 0x92, 0x0A, 0x7F, 0x7F,
    0x7F, 0x54, 0x40, 0xF0, 0x9E, 0xDD, 0xF3, 0xF3,
    0x16, 0xD8, 0xD4, 0x97, 0x80, 0xE5, 0x37, 0x2C,
    0x1D, 0x2C, 0x26, 0x3D, 0x04, 0xEA
    };

static const q7_t ref_conv_30_34[63] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xD8,
    0xA8, 0x09, 0xDE, 0xB9, 0x83, 0xFE, 0x7F, 0x7F,
    0x7F, 0x57, 0x5C, 0x11, 0xAC, 0xEB, 0xD2, 0xDD,
    0x0A, 0xDB, 0xE3, 0x85, 0x81, 0xEB, 0x19, 0xF2,
    0x06, 0x40, 0x24, 0x5B, 0x00, 0x00, 0x1F
    };

static const q7_t ref_conv_30_49[78] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xD8,
    0xA8, 0x09, 0xE0, 0xC3, 0x80, 0x02, 0x7F, 0x7F,
    0x5A, 0x3D, 0x79, 0x4C, 0x9D, 0x00, 0x22, 0xC9,
    0xF4, 0xB7, 0xA5, 0x80, 0x80, 0x1F, 0x3D, 0x1A,
    0x10, 0x7D, 0x4A, 0x50, 0xF5, 0x1C, 0x7F, 0xED,
    0x15, 0x0F, 0xB4, 0xB9, 0xB8, 0xA2, 0xB1, 0x09,
    0x00, 0x21, 0x44, 0x02, 0x24, 0x23
    };

static const q7_t ref_conv_31_31[61] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xC1,
    0x8E, 0x38, 0xA6, 0xC8, 0x80, 0x1F, 0x7F, 0x7F,
    0x7F, 0x50, 0x43, 0xE0, 0xB3, 0xD8, 0xEF, 0xEF,
    0x13, 0xEC, 0xC5, 0xA3, 0x8C, 0xF6, 0x1C, 0x10,
    0x2F, 0x18, 0x3B, 0x22, 0xF1
    };

static const q7_t ref_conv_31_32[62] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xC4,
    0xA6, 0x25, 0x9D, 0xC1, 0x80, 0x11, 0x7F, 0x7F,
    0x7F, 0x66, 0x4B, 0xE9, 0x9E, 0xCA, 0xE8, 0xF1,
    0x1D, 0xE0, 0xCB, 0xA7, 0x80, 0xD1, 0x0D, 0x1C,
    0x2E, 0x2B, 0x38, 0x30, 0x05, 0xFA
    };

static const q7_t ref_conv_31_33[63] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xC4,
    0xA2, 0x0B, 0xB2, 0xCC, 0x87, 0x07, 0x7F, 0x7F,
    0x7F, 0x53, 0x33, 0xDF, 0x94, 0xE2, 0xF7, 0xF9,
    0x1A, 0xD6, 0xD8, 0xA0, 0x80, 0xE6, 0x36, 0x2C,
    0x20, 0x2C, 0x23, 0x33, 0xF6, 0xE4, 0x07
    };

static const q7_t ref_conv_31_34[64] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xC4,
    0xA2, 0x10, 0xD7, 0xAE, 0x80, 0xFC, 0x7F, 0x7F,
    0x7F, 0x56, 0x4F, 0x01, 0xA2, 0xF0, 0xD6, 0xE4,
    0x0E, 0xD9, 0xE7, 0x8E, 0x80, 0xED, 0x18, 0xF2,
    0x09, 0x40, 0x21, 0x51, 0xF1, 0xFA, 0x26, 0xF6
    };

static const q7_t ref_conv_31_49[79] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xC4,
    0xA2, 0x10, 0xD9, 0xB7, 0x80, 0xFF, 0x7F, 0x7F,
    0x5C, 0x3B, 0x6C, 0x3B, 0x93, 0x05, 0x26, 0xCF,
    0xF8, 0xB5, 0xA9, 0x80, 0x80, 0x21, 0x3D, 0x1A,
    0x13, 0x7D, 0x47, 0x46, 0xE7, 0x15, 0x7F, 0xE3,
    0x13, 0x11, 0xB3, 0xBD, 0xB4, 0xAA, 0xBE, 0x0B,
    0xFE, 0x27, 0x44, 0xF6, 0x24, 0x20, 0xF4
    };

static const q7_t ref_conv_32_31[62] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xA9,
    0x80, 0x30, 0xB0, 0xBF, 0x80, 0x10, 0x7F, 0x7F,
    0x68, 0x52, 0x41, 0xCD, 0x9C, 0xCA, 0xF6, 0xF4,
    0x1C, 0xF2, 0xC2, 0xA8, 0x99, 0xF3, 0x1E, 0x0F,
    0x2F, 0x1C, 0x3B, 0x1E, 0xE3, 0xEB
    };

static const q7_t ref_conv_32_32[63] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0x89, 0x1C, 0xA7, 0xB7, 0x80, 0x01, 0x7F, 0x7F,
    0x79, 0x68, 0x49, 0xD6, 0x86, 0xBC, 0xEF, 0xF6,
    0x26, 0xE7, 0xC9, 0xAD, 0x86, 0xCD, 0x0F, 0x1C,
    0x2E, 0x2F, 0x38, 0x2C, 0xF7, 0xE5, 0xF7
    };

static const q7_t ref_conv_32_33[64] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0x85, 0x02, 0xBC, 0xC2, 0x80, 0xF8, 0x7F, 0x7F,
    0x77, 0x55, 0x32, 0xCD, 0x80, 0xD3, 0xFE, 0xFF,
    0x23, 0xDC, 0xD5, 0xA6, 0x81, 0xE2, 0x38, 0x2C,
    0x20, 0x31, 0x23, 0x2F, 0xE7, 0xCF, 0xFE, 0x0A
    };

static const q7_t ref_conv_32_34[65] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0x85, 0x08, 0xE1, 0xA4, 0x80, 0xEC, 0x7F, 0x7F,
    0x6D, 0x58, 0x4D, 0xEE, 0x8A, 0xE1, 0xDD, 0xE9,
    0x17, 0xE0, 0xE4, 0x93, 0x8B, 0xE9, 0x1B, 0xF2,
    0x0A, 0x44, 0x21, 0x4D, 0xE2, 0xE5, 0x1D, 0x00,
    0xF1
    };

static const q7_t ref_conv_32_49[80] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0x85, 0x08, 0xE3, 0xAD, 0x80, 0xF0, 0x7F, 0x7F,
    0x3C, 0x3D, 0x6B, 0x29, 0x80, 0xF6, 0x2C, 0xD4,
    0x01, 0xBB, 0xA6, 0x80, 0x80, 0x1D, 0x3F, 0x1A,
    0x13, 0x7F, 0x47, 0x41, 0xD8, 0x00, 0x7F, 0xED,
    0x04, 0x0D, 0xB5, 0xBA, 0xBA, 0xA4, 0xC8, 0x1E,
    0x02, 0x24, 0x4C, 0xF6, 0x13, 0x20, 0xEF, 0xEF
    };

static const q7_t ref_conv_33_31[63] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xA9,
    0xA2, 0x6C, 0xC2, 0xA9, 0x80, 0x34, 0x7F, 0x7F,
    0x45, 0x7F, 0x3D, 0xD1, 0xC3, 0xFB, 0x14, 0xE6,
    0x11, 0xDF, 0xB5, 0xAE, 0x8D, 0xD7, 0x26, 0x0B,
    0x31, 0x1C, 0x31, 0x1E, 0xEC, 0x09, 0x2C
    };

static const q7_t ref_conv_33_32[64] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0xBA, 0x59, 0xB8, 0xA2, 0x83, 0x25, 0x7F, 0x7F,
    0x56, 0x7F, 0x45, 0xDA, 0xAE, 0xED, 0x0D, 0xE8,
    0x1A, 0xD4, 0xBB, 0xB2, 0x80, 0xB2, 0x17, 0x17,
    0x2F, 0x2F, 0x2E, 0x2B, 0xFF, 0x03, 0x22, 0x13
    };

static const q7_t ref_conv_33_33[65] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0xB6, 0x3E, 0xCE, 0xAC, 0x8B, 0x1B, 0x7F, 0x7F,
    0x54, 0x7F, 0x2E, 0xD0, 0xA4, 0x05, 0x1C, 0xF0,
    0x18, 0xCA, 0xC8, 0xAB, 0x80, 0xC7, 0x40, 0x27,
    0x21, 0x30, 0x19, 0x2F, 0xF0, 0xED, 0x2A, 0x1E,
    0xEB
    };

static const q7_t ref_conv_33_34[66] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0xB6, 0x44, 0xF3, 0x8E, 0x80, 0x10, 0x7F, 0x7F,
    0x4B, 0x7F, 0x49, 0xF2, 0xB1, 0x13, 0xFB, 0xDA,
    0x0C, 0xCD, 0xD7, 0x99, 0x80, 0xCD, 0x23, 0xED,
    0x0B, 0x44, 0x17, 0x4D, 0xEB, 0x03, 0x48, 0x14,
    0xDC, 0x1E
    };

static const q7_t ref_conv_33_49[81] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0xB6, 0x44, 0xF5, 0x98, 0x80, 0x13, 0x7F, 0x7F,
    0x19, 0x7F, 0x67, 0x2C, 0xA3, 0x28, 0x4B, 0xC6,
    0xF6, 0xA8, 0x99, 0x80, 0x80, 0x01, 0x47, 0x15,
    0x14, 0x7F, 0x3D, 0x41, 0xE1, 0x1F, 0x7F, 0x00,
    0xEF, 0x2B, 0xBD, 0xB6, 0xBE, 0x98, 0xD4, 0x08,
    0xDA, 0x1C, 0x51, 0xE6, 0x13, 0x43, 0xEF, 0xFA,
    0x23
    };

static const q7_t ref_conv_48_31[78] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xA9,
    0xA2, 0x66, 0xD9, 0xD4, 0x80, 0xF0, 0x7F, 0x7F,
    0x71, 0x7B, 0xEF, 0xFB, 0x26, 0x26, 0x3B, 0xB7,
    0x43, 0x87, 0xE6, 0xCE, 0x80, 0x31, 0xF1, 0x38,
    0x0B, 0x69, 0x70, 0xEC, 0xAD, 0xE1, 0x1F, 0x26,
    0x18, 0xD4, 0xE2, 0xF2, 0x3C, 0x0F, 0xE6, 0x05,
    0xFE, 0x26, 0x3A, 0x01, 0xF8, 0xD5
    };

static const q7_t ref_conv_48_32[79] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0xBA, 0x53, 0xCF, 0xCC, 0x80, 0xE1, 0x7F, 0x7F,
    0x7F, 0x7F, 0xF8, 0x04, 0x10, 0x18, 0x33, 0xB9,
    0x4D, 0x80, 0xED, 0xD3, 0x80, 0x0C, 0xE3, 0x44,
    0x0A, 0x7C, 0x6D, 0xFA, 0xC1, 0xDA, 0x16, 0x39,
    0x16, 0xE0, 0xE6, 0xE4, 0x36, 0x10, 0xF7, 0xFA,
    0x02, 0x25, 0x35, 0x1A, 0xF6, 0xDF, 0xED
    };

static const q7_t ref_conv_48_33[80] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0xB6, 0x39, 0xE4, 0xD7, 0x80, 0xD8, 0x7F, 0x7F,
    0x7F, 0x7D, 0xE0, 0xFB, 0x07, 0x30, 0x43, 0xC1,
    0x4A, 0x80, 0xF9, 0xCC, 0x80, 0x21, 0x0C, 0x54,
    0xFC, 0x7E, 0x58, 0xFE, 0xB2, 0xC5, 0x1D, 0x44,
    0x00, 0xE2, 0xD9, 0xE1, 0x45, 0x17, 0xF6, 0xE7,
    0x0E, 0x20, 0x37, 0x1F, 0xDA, 0xE2, 0xE2, 0x15
    };

static const q7_t ref_conv_48_34[81] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0xB6, 0x3E, 0x0A, 0xB9, 0x80, 0xCC, 0x7F, 0x7F,
    0x76, 0x7F, 0xFB, 0x1C, 0x14, 0x3E, 0x22, 0xAC,
    0x3F, 0x80, 0x08, 0xB9, 0x80, 0x27, 0xEE, 0x1A,
    0xE5, 0x7F, 0x56, 0x1B, 0xAD, 0xDA, 0x3C, 0x3A,
    0xF2, 0x01, 0xD5, 0xF3, 0x4A, 0x01, 0xED, 0xE9,
    0x29, 0x0F, 0x3D, 0x1D, 0xD3, 0x09, 0xDE, 0x25,
    0xE2
    };

static const q7_t ref_conv_48_49[96] = {
    0x09, 0x48, 0x1D, 0xBE, 0xA8, 0x2C, 0x18, 0xCF,
    0xA6, 0x02, 0x7F, 0x1B, 0x10, 0xEB, 0x25, 0xA7,
    0xCF, 0x0A, 0xE0, 0xD5, 0xE2, 0x4B, 0x85, 0xCC,
    0x1A, 0x13, 0xC8, 0x80, 0xCC, 0x7F, 0x7F, 0xAD,
    0xB6, 0x3E, 0x0B, 0xC2, 0x80, 0xCF, 0x7F, 0x7F,
    0x45, 0x66, 0x19, 0x57, 0x05, 0x53, 0x71, 0x97,
    0x28, 0x80, 0xCA, 0x80, 0x80, 0x5B, 0x12, 0x42,
    0xEF, 0x7F, 0x7C, 0x10, 0xA3, 0xF6, 0x7F, 0x26,
    0x04, 0x0E, 0x93, 0xA8, 0x0E, 0x98, 0xBE, 0xEA,
    0x09, 0x32, 0x67, 0x09, 0xFD, 0x63, 0xB0, 0x22,
    0x25, 0xCE, 0x18, 0xEB, 0xE4, 0xC7, 0x21, 0x23,
    0xF4, 0x27, 0x07, 0x19, 0x09, 0x01, 0x07, 0xDE
    };

static const q7_t ref_conv_partial_3_6_8[4] = {
    0x10, 0xE5, 0xE5, 0xF1
    };

static const q7_t ref_conv_partial_9_6_8[4] = {
    0xEF, 0xF4, 0xEF, 0x17
    };

static const q7_t ref_conv_partial_7_6_8[4] = {
    0x27, 0x15, 0xEF, 0xF4
    };

