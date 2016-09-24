/****************************************************************************
 * Copyright (C) 2015 Haltian Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Dmitry Nikolaev <dmitry.nikolaev@haltian.com>
 *
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>

#include "qr_simple_bch.h"
#include "qr_simple_fill_matrix.h"
#include "qr_simple_render.h"

#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>

#ifdef CONFIG_THINGSEE_DISPLAY_TRACES
#  define lcd_dbg(x, ...)    dbg(x, ##__VA_ARGS__)
#  define lcd_lldbg(x, ...)  lldbg(x, ##__VA_ARGS__)
#else
#  define lcd_dbg(x, ...)
#  define lcd_lldbg(x, ...)
#endif

/* TO DO: clean up code!!!!
 * TO DO: Well, mask patterns evaluation must be done according to http://www.thonky.com/qr-code-tutorial/data-masking/
 * But picking up a random mask and applying it, will not have much effect on the final result (besides making it more readable)
 * on the symbol. Thus I'll pick up a mask number one, which stays every second row must be masked. */

/* Defined in version documentation */
#define QR_SIMPLE_WORDS_NBR         44

/* Side length in pixels (according to the documentation 1 module equals 1 pixel)
 * Since we have 25x25 matrix */
#define QR_SIMPLE_SIDE_HEIGHT       25
#define QR_SIMPLE_SIDE_WIDTH        25

#define QR_SIMPLE_ALIGN_START_X     4
#define QR_SIMPLE_ALIGN_START_Y     4
#define QR_SIMPLE_ALIGN_LENGTH      5

#define QR_SIMPLE_POS_SIDE_LEN      7

#define QR_SIMPLE_POS_1_X           0
#define QR_SIMPLE_POS_1_Y           18

#define QR_SIMPLE_POS_2_X           18
#define QR_SIMPLE_POS_2_Y           18

#define QR_SIMPLE_POS_3_X           18
#define QR_SIMPLE_POS_3_Y           0


#define QR_SIMPLE_SEP_SIDE          8

#define QR_SIMPLE_SEP_1_X           0
#define QR_SIMPLE_SEP_1_Y           17

#define QR_SIMPLE_SEP_2_X           17
#define QR_SIMPLE_SEP_2_Y           17

#define QR_SIMPLE_SEP_3_X           17
#define QR_SIMPLE_SEP_3_Y           0


#define QR_SIMPLE_VER_HEIGHT        9
#define QR_SIMPLE_VER_WIDTH         8

#define QR_SIMPLE_VER_1_X           0
#define QR_SIMPLE_VER_1_Y           16

#define QR_SIMPLE_VER_2_X           16
#define QR_SIMPLE_VER_2_Y           16

#define QR_SIMPLE_VER_3_X           16
#define QR_SIMPLE_VER_3_Y           0


#define QR_SIMPLE_TIME1_WIDTH       9
#define QR_SIMPLE_TIME1_HEIGHT      1
#define QR_SIMPLE_TIME2_WIDTH       1
#define QR_SIMPLE_TIME2_HEIGHT      9

#define QR_SIMPLE_TIME_1_X          8
#define QR_SIMPLE_TIME_1_Y          18
#define QR_SIMPLE_TIME_2_X          18
#define QR_SIMPLE_TIME_2_Y          8

/* By default we are using L-level of ECC */
#define QR_SIMPLE_L_INDICATOR       0x1

/* Names for those variables selected properly
 * Basically one byte is one bit in matrix and it will be used so */

#define QR_SIMPLE_EMPTY_BIT         0x0
#define QR_SIMPLE_FULL_BIT          0x01
#define QR_SIMPLE_ALIGN_BIT         0x03
#define QR_SIMPLE_POS_BIT           0x04
#define QR_SIMPLE_TIME_BIT          0x05
#define QR_SIMPLE_SEPARATOR_BIT     0x06
#define QR_SIMPLE_VERSION_BIT       0x07
#define QR_SIMPLE_INIT_BIT          0xFF


static uint8_t g_matrix[QR_SIMPLE_SIDE_HEIGHT][QR_SIMPLE_SIDE_WIDTH];


static void qr_simple_fill_alignment_pattern(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t i, j;

    /* fill place for alignment pattern */

    for (i = QR_SIMPLE_ALIGN_START_Y; i < (QR_SIMPLE_ALIGN_LENGTH + QR_SIMPLE_ALIGN_START_X); i++) {
        for (j = QR_SIMPLE_ALIGN_START_X; j < (QR_SIMPLE_ALIGN_LENGTH + QR_SIMPLE_ALIGN_START_Y); j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_ALIGN_BIT;
        }
    }
}

static void qr_simple_fill_version_patterns(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t i, j;

    /* fill version block 1 */

    for (i = QR_SIMPLE_VER_1_Y; i < QR_SIMPLE_SIDE_HEIGHT; i++) {
        for (j = QR_SIMPLE_VER_1_X; j < QR_SIMPLE_VER_WIDTH; j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_VERSION_BIT;
        }
    }

    /* fill version block 2 */

    for (i = QR_SIMPLE_VER_2_Y; i < (QR_SIMPLE_VER_2_Y + QR_SIMPLE_VER_HEIGHT); i++) {
        for (j = QR_SIMPLE_VER_2_X; j < (QR_SIMPLE_VER_2_X + QR_SIMPLE_VER_HEIGHT); j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_VERSION_BIT;
        }
    }

    /* fill version block 3 */

    for (i = QR_SIMPLE_VER_3_Y; i < (QR_SIMPLE_VER_3_Y + QR_SIMPLE_VER_WIDTH); i++) {
        for (j = QR_SIMPLE_VER_3_X; j < (QR_SIMPLE_SEP_3_X + QR_SIMPLE_VER_WIDTH); j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_VERSION_BIT;
        }
    }
}

static void qr_simple_fill_separator_patterns(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t i, j;

    /* fill separator block 1 */

    for (i = QR_SIMPLE_SEP_1_Y; i < QR_SIMPLE_SIDE_HEIGHT; i++) {
        for (j = QR_SIMPLE_SEP_1_X; j < QR_SIMPLE_SEP_SIDE; j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_SEPARATOR_BIT;
        }
    }

    /* fill separator block 2 */

    for (i = QR_SIMPLE_SEP_2_Y; i < (QR_SIMPLE_SEP_2_Y + QR_SIMPLE_SEP_SIDE); i++) {
        for (j = QR_SIMPLE_SEP_2_X; j < (QR_SIMPLE_SEP_2_X + QR_SIMPLE_SEP_SIDE); j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_SEPARATOR_BIT;
        }
    }

    /* fill separator block 3 */

    for (i = QR_SIMPLE_SEP_3_Y; i < (QR_SIMPLE_SEP_3_Y + QR_SIMPLE_SEP_SIDE); i++) {
        for (j = QR_SIMPLE_SEP_3_X; j < (QR_SIMPLE_SEP_3_X + QR_SIMPLE_SEP_SIDE); j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_SEPARATOR_BIT;
        }
    }
}

static void qr_simple_fill_position_patterns(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t i, j;

    /* fill position block 1 */

    for (i = QR_SIMPLE_POS_1_Y; i < QR_SIMPLE_SIDE_HEIGHT; i++) {
        for (j = QR_SIMPLE_POS_1_X; j < QR_SIMPLE_POS_SIDE_LEN; j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_POS_BIT;
        }
    }

    /* fill position block 2 */

    for (i = QR_SIMPLE_POS_2_Y; i < (QR_SIMPLE_POS_2_Y + QR_SIMPLE_POS_SIDE_LEN); i++) {
        for (j = QR_SIMPLE_POS_2_X; j < (QR_SIMPLE_POS_2_X + QR_SIMPLE_POS_SIDE_LEN); j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_POS_BIT;
        }
    }

    /* fill position block 3 */

    for (i = QR_SIMPLE_POS_3_Y; i < (QR_SIMPLE_POS_3_Y + QR_SIMPLE_POS_SIDE_LEN); i++) {
        for (j = QR_SIMPLE_POS_3_X; j < (QR_SIMPLE_POS_3_X + QR_SIMPLE_POS_SIDE_LEN); j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_POS_BIT;
        }
    }
}

static void qr_simple_place_time_patterns(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t i, j;

    /* fill time block 1 */

    for (i = QR_SIMPLE_TIME_1_Y; i < (QR_SIMPLE_TIME_1_Y + QR_SIMPLE_TIME1_HEIGHT); i++) {
        for (j = QR_SIMPLE_TIME_1_X; j < (QR_SIMPLE_TIME_1_X + QR_SIMPLE_TIME1_WIDTH); j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_TIME_BIT;
        }
    }

    /* fill time block 2 */

    for (i = QR_SIMPLE_TIME_2_Y; i < (QR_SIMPLE_TIME_2_Y + QR_SIMPLE_TIME2_HEIGHT); i++) {
        for (j = QR_SIMPLE_TIME_2_X; j < (QR_SIMPLE_TIME_2_X + QR_SIMPLE_TIME2_WIDTH); j++) {
            *(*(matrix + i) + j) = QR_SIMPLE_TIME_BIT;
        }
    }
}

static void qr_simple_place_function_patterns(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    qr_simple_fill_alignment_pattern(matrix);
    qr_simple_fill_version_patterns(matrix);
    qr_simple_fill_separator_patterns(matrix);
    qr_simple_fill_position_patterns(matrix);
    qr_simple_place_time_patterns(matrix);
}

static void qr_simple_checkadd_right_bit(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t bit_pair, bool do_reset)
{
    static uint8_t col = 0;
    static int8_t row = 0;
    static bool is_up = true;
    uint8_t bit = 0;
    uint8_t cell = 0;

    if (do_reset) {
    	col = 0;
    	row = 0;
    	is_up = true;
    	return;
    }

    bit = (bit_pair >> 1) & 0x1;
    cell = *(*(matrix + row) + col);

    switch (cell) {
    case QR_SIMPLE_INIT_BIT:
        *(*(matrix + row) + col) = bit;
        break;
    case QR_SIMPLE_VERSION_BIT:
        cell = *(*(matrix + row + 1) + col);
        if (cell != QR_SIMPLE_VERSION_BIT) {
            cell = *(*(matrix + row) + col + 1);

            col = ((cell == QR_SIMPLE_TIME_BIT) ? (col + 3) : (col + 2));
            row--;
            is_up = false;
        } else {
            row += QR_SIMPLE_VER_WIDTH;
            cell = *(*(matrix + row) + col);
            if (cell != QR_SIMPLE_INIT_BIT) {
                col++;
                *(*(matrix + row) + col) = bit;
            }
        }
        *(*(matrix + row) + col) = bit;
        break;
    case QR_SIMPLE_ALIGN_BIT:
        /* First, check if the left cell is occupied as well */
        if (*(*(matrix + row) + col + 1) == QR_SIMPLE_ALIGN_BIT) {
            row = (is_up ? (row + 5) : (row - 5));
            *(*(matrix + row) + col) = bit;
        } else if (*(*(matrix + row) + col + 1) == QR_SIMPLE_FULL_BIT
                   || *(*(matrix + row) + col + 1) == QR_SIMPLE_EMPTY_BIT) {
            row++;
            *(*(matrix + row) + col + 1) = bit;
        } else if (*(*(matrix + row) + col + 1) == QR_SIMPLE_INIT_BIT) {
            *(*(matrix + row) + col + 1) = bit;
        }
        break;
    case QR_SIMPLE_TIME_BIT:
        row = (is_up ? (row + 1) : (row -1));
        *(*(matrix + row) + col) = bit;
        break;
    case QR_SIMPLE_SEPARATOR_BIT:
        is_up = true;   /* The only possible direction here */
        col += 2;
        row++;
        *(*(matrix + row) + col) = bit;
        break;
    case QR_SIMPLE_FULL_BIT:
    case QR_SIMPLE_EMPTY_BIT:
        *(*(matrix + row) + col + 1) = bit;
        break;
    default:
        lcd_dbg("Right bit cannot be here");
        break;
    }

    row = (is_up ? (row + 1) : (row - 1));

    if (row == QR_SIMPLE_SIDE_HEIGHT) {
        is_up = false;
        col += 2;
        row--;
    } else if (row < 0) {
        is_up = true;
        col += 2;
        row++;
    }
}

static void qr_simple_checkadd_left_bit(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t bit_pair, bool do_reset)
{
    static uint8_t col = 1;
    static int8_t row = 0;
    static bool is_up = true;
    uint8_t bit = 0;
    uint8_t cell = 0;

    if (do_reset) {
    	col = 1;
    	row = 0;
    	is_up = true;
    	return;
    }

    cell = *(*(matrix + row) + col);
    bit = bit_pair & 0x1;

    switch (cell) {
    case QR_SIMPLE_INIT_BIT:
        cell = *(*(matrix + row) + col - 1);
        if (cell == QR_SIMPLE_INIT_BIT) {
            *(*(matrix + row) + col - 1) = bit;
        } else {
            *(*(matrix + row) + col) = bit;
        }
        break;
    case QR_SIMPLE_VERSION_BIT:
        cell = *(*(matrix + row + 1) + col);
        if (cell != QR_SIMPLE_VERSION_BIT) {
            row--;
            col += 2;
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_TIME_BIT) {
                col++;
            }
            is_up = false;
        } else {
            row += QR_SIMPLE_VER_WIDTH;
        }
        *(*(matrix + row) + col) = bit;
        break;
    case QR_SIMPLE_ALIGN_BIT:
        row = (is_up ? (row + 5) : (row - 5));
        *(*(matrix + row) + col) = bit;
        break;
    case QR_SIMPLE_TIME_BIT:
        row = (is_up ? (row + 1) : (row -1));
        *(*(matrix + row) + col) = bit;
        break;
    case QR_SIMPLE_FULL_BIT:
    case QR_SIMPLE_EMPTY_BIT:
        cell = *(*(matrix + row) + col - 1);
        if (cell == QR_SIMPLE_ALIGN_BIT) {
            row++;
        }
        *(*(matrix + row) + col) = bit;
        break;
    case QR_SIMPLE_SEPARATOR_BIT:
    {
        uint8_t second_cell = 0;
        cell = *(*(matrix + row + 1) + col);
        second_cell = *(*(matrix + row - 1) + col);
        if (cell == QR_SIMPLE_SEPARATOR_BIT && second_cell != QR_SIMPLE_POS_BIT) {
            row += QR_SIMPLE_SEP_SIDE;
            *(*(matrix + row) + col - 1) = bit;
        } else if (second_cell == QR_SIMPLE_POS_BIT) {
            is_up = true;
            col += 2;
            row++;
            *(*(matrix + row) + col) = bit;
        }
    }
        break;
    default:
        lcd_dbg("Left bit cannot be here");
        break;
    }

    row = (is_up ? (row + 1) : (row - 1));

    if (row == QR_SIMPLE_SIDE_HEIGHT) {
        is_up = false;
        col += 2;
        row--;
    } else if (row < 0) {
        is_up = true;
        col += 2;
        row++;
    }
}

static void qr_simple_add_data_pair(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t bit_pair)
{
    qr_simple_checkadd_right_bit(matrix, bit_pair, false);
    qr_simple_checkadd_left_bit(matrix, bit_pair, false);
}

static void qr_simple_place_data_modules(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t *data_modules)
{
    uint8_t word_index;
    uint8_t bit_pair = 0;
    int8_t pair_counter;
    uint8_t row;


    for (word_index = 0; word_index < 44; word_index++) {
        for (pair_counter = 6; pair_counter >= 0; pair_counter -= 2) {
            bit_pair = *(data_modules + word_index) >> pair_counter;
            qr_simple_add_data_pair(matrix, bit_pair);
        }
    }

    /* There are 7 remainder bits. Let's fill them with data */
    for (row = 8; row < 11; row++) {
        *(*(matrix + row) + 23) = QR_SIMPLE_EMPTY_BIT;
    }

    for (row = 8; row < 12; row++) {
        *(*(matrix + row) + 24) = QR_SIMPLE_EMPTY_BIT;
    }

    qr_simple_checkadd_right_bit(NULL, 0, true);
    qr_simple_checkadd_left_bit(NULL, 0, true);
}

#ifndef CONFIG_THINGSEE_MASKING_OFF

static uint32_t qr_simple_check_horline(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t row, uint8_t *col)
{
    uint32_t score = 0;
    uint8_t shift = 1;

    uint8_t cur_cell = QR_SIMPLE_INIT_BIT;

    cur_cell = *(*(matrix + row) + *col);


    while (cur_cell == *(*(matrix + row) + *col + shift) && (*col + shift) < 25) {
        shift++;
    }

    *col += shift - 1;

    if (shift >= 5) {
        score = 3 + shift - 5;
    }

    return score;
}

static uint32_t qr_simple_check_vertline(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t *row, uint8_t col)
{
    uint32_t score = 0;
    uint8_t shift = 1;
    uint8_t cur_cell = QR_SIMPLE_INIT_BIT;

    cur_cell = *(*(matrix + *row) + col);

    while (cur_cell == *(*(matrix + *row + shift) + col) && (*row + shift) < 25) {
        shift++;
    }

    *row += shift - 1;

    if (shift >= 5) {
        score = 3 + shift - 5;
    }

    return score;
}

static uint32_t qr_simple_check_adjacent(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint32_t score = 0;
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    for (row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                /* Horizontal line */
                score += qr_simple_check_horline(matrix, row, &col);
            }
        }
    }

    for (col = 0; col < 25; col++) {
        for (row = 0; row < 25; row++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                /* Vertical line */
                score += qr_simple_check_vertline(matrix, &row, col);
            }
        }
    }

    return score;
}

static uint8_t qr_simple_find_block(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t row, uint8_t col, uint8_t cell)
{
    /* Instead of searching for the entire block
     * the minimal size block will be applied.
     * If block is found, 3 scores is returned.
     * Otherwise 0. Thus we do not need another
     * table to place a result */

    const uint8_t score = 3;

    if (cell == *(*(matrix + row) + col + 1)
        && cell == *(*(matrix + row + 1) + col)
        && cell == *(*(matrix + row + 1) + col +1)) {
        return score;
    } else {
        return 0;
    }

}

static uint32_t qr_simple_block_modules(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint32_t score = 0;
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    for (row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                score += qr_simple_find_block(matrix, row, col, cell);
            }
        }
    }

    return score;
}

static uint8_t qr_simple_find_horpattern(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t row, uint8_t *col)
{
    uint8_t shift_x;
    uint8_t byte = 0;
    const uint8_t pattern = 0x5D;
    const uint8_t pattern_length = 7;
    const uint8_t penalty_score = 40;

    if ((*col + pattern_length) > QR_SIMPLE_SIDE_WIDTH) {
        return 0;
    }


    for (shift_x = 0; shift_x < pattern_length; shift_x++) {
        byte |= *(*(matrix + row) + *col + shift_x) << shift_x;
    }

    if (byte == pattern) {
        *col += pattern_length - 1;
        return penalty_score;
    } else {
        return 0;
    }
}

static uint8_t qr_simple_find_vertpattern(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t *row, uint8_t col)
{
    uint8_t shift_y;
    uint8_t byte = 0;
    const uint8_t pattern = 0x5D;
    const uint8_t pattern_length = 7;
    const uint8_t penalty_score = 40;

    if ((*row + pattern_length) > QR_SIMPLE_SIDE_HEIGHT) {
        return 0;
    }

    for (shift_y = 0; shift_y < pattern_length; shift_y++) {
        byte |= *(*(matrix + *row + shift_y) + col) << shift_y;
    }

    if (byte == pattern) {
        *row += pattern_length - 1;
        return penalty_score;
    } else {
        return 0;
    }
}

static uint32_t qr_simple_find_pos_pattern(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;
    uint32_t score = 0;

    /* Horizontal pattern search */
    for (row = 0; row < QR_SIMPLE_SIDE_HEIGHT; row++) {
        /* the pattern is 7 bits long. To avoid addressing outside
         * of the array, the maximum right value is reduced by 7 */
        for (col = 0; col < (QR_SIMPLE_SIDE_WIDTH - 7); col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                score += qr_simple_find_horpattern(matrix, row, &col);
            }
        }
    }

    /* Vertical pattern search */
    for (col = 0; col < QR_SIMPLE_SIDE_WIDTH; col++) {
        /* the pattern is 7 bits long. To avoid addressing outside
         * of the array, the maximum right value is reduced by 7 */
        for (row = 0; row < (QR_SIMPLE_SIDE_HEIGHT - 7); row++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                score += qr_simple_find_vertpattern(matrix, &row, col);
            }
        }
    }

    return score;
}

static uint32_t qr_simple_proportion_dark(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint32_t score = 0;
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;
    uint16_t cell_counter = 0;
    uint16_t dark_counter = 0;
    uint16_t dark_proportion = 0;
    uint16_t dark_coef = 0;
    const uint16_t max_dark_value = 500;
    const uint16_t score_nbr = 10;

    for (row = 0; row < QR_SIMPLE_SIDE_HEIGHT; row++) {
        for (col = 0; col < QR_SIMPLE_SIDE_WIDTH; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                cell_counter++;
                if (cell == QR_SIMPLE_FULL_BIT) {
                    dark_counter++;
                }
            }
        }
    }

    dark_proportion = (dark_counter * 1000) / cell_counter;

    if (dark_proportion >= max_dark_value) {
        dark_coef = (dark_proportion - max_dark_value) / 10;
        score = (dark_coef ? (dark_coef * score_nbr) : score_nbr);
    } else {
        score = 0;
    }

    return score;
}

static uint32_t qr_simple_0_mask(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint32_t scores = 0;
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!((row + col) % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }

    /* Let's check adjacent cells */
    scores = qr_simple_check_adjacent(matrix);
    /* Let's check blocks of the same color */
    scores += qr_simple_block_modules(matrix);
    /* Let's find out how many 1011101 patterns is presented
     * in table */
    scores += qr_simple_find_pos_pattern(matrix);
    /* Let's check dark modules number */
    scores += qr_simple_proportion_dark(matrix);

    /* Let's re-apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!((row + col) % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }

    lcd_dbg("Checked with scores: %d\n", scores);

    return scores;
}

static uint32_t qr_simple_1_mask(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint32_t scores = 0;
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(row % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }

    /* Let's check adjacent cells */
    scores = qr_simple_check_adjacent(matrix);
    /* Let's check blocks of the same color */
    scores += qr_simple_block_modules(matrix);
    /* Let's find out how many 1011101 patterns is presented
     * in table */
    scores += qr_simple_find_pos_pattern(matrix);
    /* Let's check dark modules number */
    scores += qr_simple_proportion_dark(matrix);

    /* Let's re-apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(row % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }

    lcd_dbg("Checked with scores: %d\n", scores);

    return scores;
}

static uint32_t qr_simple_2_mask(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint32_t scores = 0;
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(col % 3) ? (cell ^ 0x1) : cell);
            }
        }
    }

    /* Let's check adjacent cells */
    scores = qr_simple_check_adjacent(matrix);
    /* Let's check blocks of the same color */
    scores += qr_simple_block_modules(matrix);
    /* Let's find out how many 1011101 patterns is presented
     * in table */
    scores += qr_simple_find_pos_pattern(matrix);
    /* Let's check dark modules number */
    scores += qr_simple_proportion_dark(matrix);

    /* Let's re-apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(col % 3) ? (cell ^ 0x1) : cell);
            }
        }
    }

    lcd_dbg("Checked with scores: %d\n", scores);

    return scores;
}

static uint32_t qr_simple_3_mask(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint32_t scores = 0;
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!((col + row) % 3) ? (cell ^ 0x1) : cell);
            }
        }
    }

    /* Let's check adjacent cells */
    scores = qr_simple_check_adjacent(matrix);
    /* Let's check blocks of the same color */
    scores += qr_simple_block_modules(matrix);
    /* Let's find out how many 1011101 patterns is presented
     * in table */
    scores += qr_simple_find_pos_pattern(matrix);
    /* Let's check dark modules number */
    scores += qr_simple_proportion_dark(matrix);

    /* Let's re-apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!((col + row) % 3) ? (cell ^ 0x1) : cell);
            }
        }
    }

    lcd_dbg("Checked with scores: %d\n", scores);

    return scores;
}

static uint32_t qr_simple_4_mask(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint32_t scores = 0;
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(((row / 2) + (col / 3)) % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }

    /* Let's check adjacent cells */
    scores = qr_simple_check_adjacent(matrix);
    /* Let's check blocks of the same color */
    scores += qr_simple_block_modules(matrix);
    /* Let's find out how many 1011101 patterns is presented
     * in table */
    scores += qr_simple_find_pos_pattern(matrix);
    /* Let's check dark modules number */
    scores += qr_simple_proportion_dark(matrix);

    /* Let's re-apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(((row / 2) + (col / 3)) % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }

    lcd_dbg("Checked with scores: %d\n", scores);

    return scores;
}

static uint32_t qr_simple_5_mask(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint32_t scores = 0;
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!((row * col) % 2 + (col * row) % 3) ? (cell ^ 0x1) : cell);
            }
        }
    }

    /* Let's check adjacent cells */
    scores = qr_simple_check_adjacent(matrix);
    /* Let's check blocks of the same color */
    scores += qr_simple_block_modules(matrix);
    /* Let's find out how many 1011101 patterns is presented
     * in table */
    scores += qr_simple_find_pos_pattern(matrix);
    /* Let's check dark modules number */
    scores += qr_simple_proportion_dark(matrix);

    /* Let's re-apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!((row * col) % 2 + (col * row) % 3) ? (cell ^ 0x1) : cell);
            }
        }
    }

    lcd_dbg("Checked with scores: %d\n", scores);

    return scores;
}

static uint32_t qr_simple_6_mask(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint32_t scores = 0;
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(((row * col) % 2 + (col * row) % 3) % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }

    /* Let's check adjacent cells */
    scores = qr_simple_check_adjacent(matrix);
    /* Let's check blocks of the same color */
    scores += qr_simple_block_modules(matrix);
    /* Let's find out how many 1011101 patterns is presented
     * in table */
    scores += qr_simple_find_pos_pattern(matrix);
    /* Let's check dark modules number */
    scores += qr_simple_proportion_dark(matrix);

    /* Let's re-apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(((row * col) % 2 + (col * row) % 3) % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }

    lcd_dbg("Checked with scores: %d\n", scores);

    return scores;
}

static uint32_t qr_simple_7_mask(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint32_t scores = 0;
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(((row * col) % 3 + (col + row) % 2) % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }

    /* Let's check adjacent cells */
    scores = qr_simple_check_adjacent(matrix);
    /* Let's check blocks of the same color */
    scores += qr_simple_block_modules(matrix);
    /* Let's find out how many 1011101 patterns is presented
     * in table */
    scores += qr_simple_find_pos_pattern(matrix);
    /* Let's check dark modules number */
    scores += qr_simple_proportion_dark(matrix);

    /* Let's re-apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(((row * col) % 3 + (col + row) % 2) % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }

    lcd_dbg("Checked with scores: %d\n", scores);

    return scores;
}

#endif

/* Well, code will be cleaned up later. For now it will be provided as it is */

static void qr_simple_apply_mask0(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!((row + col) % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }
}

static void qr_simple_apply_mask1(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(row % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }
}

static void qr_simple_apply_mask2(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(col % 3) ? (cell ^ 0x1) : cell);
            }
        }
    }
}

static void qr_simple_apply_mask3(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!((col + row) % 3) ? (cell ^ 0x1) : cell);
            }
        }
    }
}

static void qr_simple_apply_mask4(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(((row / 2) + (col / 3)) % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }
}

static void qr_simple_apply_mask5(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!((row * col) % 2 + (col * row) % 3) ? (cell ^ 0x1) : cell);
            }
        }
    }
}

static void qr_simple_apply_mask6(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                *(*(matrix + row) + col) = (!(((row * col) % 2 + (col * row) % 3) % 2) ? (cell ^ 0x1) : cell);
            }
        }
    }
}

static void qr_simple_apply_mask7(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t row, col;
    uint8_t cell = QR_SIMPLE_INIT_BIT;

    /* Let's apply the mask */
    for(row = 0; row < 25; row++) {
        for (col = 0; col < 25; col++) {
            cell = *(*(matrix + row) + col);
            if (cell == QR_SIMPLE_EMPTY_BIT || cell == QR_SIMPLE_FULL_BIT) {
                /* According to this one http://www.thonky.com/qr-code-tutorial/mask-patterns/
                 * there is an error in specification */
                *(*(matrix + row) + col) = (!( ((row + col) % 2) + ((row + col) % 3) % 2 ) ? (cell ^ 0x1) : cell);
            }
        }
    }
}

static void qr_simple_apply_mask(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t counter)
{
    switch (counter) {
    case 0:
        qr_simple_apply_mask0(matrix);
        break;
    case 1:
        qr_simple_apply_mask1(matrix);
        break;
    case 2:
        qr_simple_apply_mask2(matrix);
        break;
    case 3:
        qr_simple_apply_mask3(matrix);
        break;
    case 4:
        qr_simple_apply_mask4(matrix);
        break;
    case 5:
        qr_simple_apply_mask5(matrix);
        break;
    case 6:
        qr_simple_apply_mask6(matrix);
        break;
    case 7:
        qr_simple_apply_mask7(matrix);
        break;
    default:
        lcd_dbg("Counter cannot be here");
        break;
    }
}

static void qr_simple_insert_format_info(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint16_t value_ecc)
{
    uint8_t i;
    uint8_t msb = 0;
    uint8_t lsb = 0;
    uint8_t col;
    int8_t row;

    lsb = value_ecc & 0xFF;
    msb = (value_ecc & 0xFF00) >> 8;

    lcd_dbg("Msb: %x, Lsb: %x\n", msb, lsb);

    /* Insert under the first position pattern */
    for (i = 0, col = 0; i < 8; i++, col++) {
        *(*(matrix + 16) + col) = (lsb >> i) & 0x1;
    }

    /* Insert near by the third position pattern */
    for (i = 0, row = 6; row >= 0; i++, row--) {
        *(*(matrix + row) + 16) = (msb >> i) & 0x1;
    }

    /* Insert to the left from the second position pattern */
    for (i = 0, col = 17; col < 25; i++, col++) {
        if (*(*(matrix + 16) + col) == QR_SIMPLE_TIME_BIT) {
            col++;
        }
        *(*(matrix + 16) + col) = (msb >> i) & 0x1;
    }

    /* Insert to the left from the second position pattern */
    for (i = 0, row = 24; i < 8; i++, row--) {
        if (*(*(matrix + row) + 16) == QR_SIMPLE_TIME_BIT) {
            row--;
        }
        *(*(matrix + row) + 16) = (lsb >> i) & 0x1;
    }

}

static void qr_simple_place_format(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH], uint8_t counter)
{
    uint8_t value = 0;
    uint16_t value_ecc = 0;

    value = (QR_SIMPLE_L_INDICATOR << 3) | counter;

    value_ecc = qr_simple_calculate_bch(value);

    lcd_dbg("Value and ECC: 0x%X Counter: %d\n", value_ecc, counter);

    qr_simple_apply_mask(matrix, counter);

    qr_simple_insert_format_info(matrix, value_ecc);
}

static void qr_simple_apply_masks(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
#ifndef CONFIG_THINGSEE_MASKING_OFF
    uint32_t score[8] = { 0 };
    uint32_t min_score = 0;
    uint32_t min_counter = 0;
    uint8_t i = 0;

    score[0] = qr_simple_0_mask(matrix);
    /* Have to do this one here. We do not know upper score beforehand */
    min_score = score[0];
    score[1] = qr_simple_1_mask(matrix);
    score[2] = qr_simple_2_mask(matrix);
    score[3] = qr_simple_3_mask(matrix);
    score[4] = qr_simple_4_mask(matrix);
    score[5] = qr_simple_5_mask(matrix);
    score[6] = qr_simple_6_mask(matrix);
    score[7] = qr_simple_7_mask(matrix);

    for (i = 0; i < 8; i++) {
        if (min_score > score[i]) {
            min_score = score[i];
            min_counter = i;
        }
    }

    lcd_dbg("Min score: %d, min counter: %d\n", min_score, min_counter);
    qr_simple_place_format(matrix, min_counter);
#else
    /* Hard-coded mask is used here to speed up masking process and it seems to work.
     * There is only problem with such approach, in noisy environment as we have here
     * decoder might fail to read it, but I'll give to it a try */
    qr_simple_place_format(matrix, 1);
#endif
}

static void qr_simple_fill_patterns(uint8_t (*matrix)[QR_SIMPLE_SIDE_WIDTH])
{
    uint8_t row, col;
    uint8_t i = 0;
    uint8_t bit = QR_SIMPLE_INIT_BIT;

    /* Let's fill aligned pattern with real data */
    while (i < 3) {
        if (i % 2) {
            bit = QR_SIMPLE_EMPTY_BIT;
        } else {
            bit = QR_SIMPLE_FULL_BIT;
        }

        for (row = QR_SIMPLE_ALIGN_START_Y + i; row < (QR_SIMPLE_ALIGN_LENGTH + QR_SIMPLE_ALIGN_START_Y - i); row++) {
            for (col = QR_SIMPLE_ALIGN_START_X + i; col < (QR_SIMPLE_ALIGN_LENGTH + QR_SIMPLE_ALIGN_START_X - i); col++) {
                *(*(matrix + row) + col) = bit;
            }
        }
        i++;
    }

    /* Let's fill separators with zeros */

    for (row = 0; row < QR_SIMPLE_SIDE_HEIGHT; row++) {
        for(col = 0; col < QR_SIMPLE_SIDE_WIDTH; col++) {
            if (*(*(matrix + row) + col) == QR_SIMPLE_SEPARATOR_BIT) {
                *(*(matrix + row) + col) = QR_SIMPLE_EMPTY_BIT;
            } else if (*(*(matrix + row) + col) == QR_SIMPLE_VERSION_BIT) {
                *(*(matrix + row) + col) = QR_SIMPLE_FULL_BIT;
            }
        }
    }

    /* Let's fill first poisition block */

    i = 0;
    while (i < 3) {
        if (i % 2) {
            bit = QR_SIMPLE_EMPTY_BIT;
        } else {
            bit = QR_SIMPLE_FULL_BIT;
        }

        for (row = QR_SIMPLE_POS_1_Y + i; row < (QR_SIMPLE_POS_SIDE_LEN + QR_SIMPLE_POS_1_Y - i); row++) {
            for (col = QR_SIMPLE_POS_1_X + i; col < (QR_SIMPLE_POS_SIDE_LEN + QR_SIMPLE_POS_1_X - i); col++) {
                *(*(matrix + row) + col) = bit;
            }
        }
        i++;
    }

    /* Let's fill second poisition block */

    i = 0;
    while (i < 3) {
        if (i % 2) {
            bit = QR_SIMPLE_EMPTY_BIT;
        } else {
            bit = QR_SIMPLE_FULL_BIT;
        }

        for (row = QR_SIMPLE_POS_2_Y + i; row < (QR_SIMPLE_POS_SIDE_LEN + QR_SIMPLE_POS_2_Y - i); row++) {
            for (col = QR_SIMPLE_POS_2_X + i; col < (QR_SIMPLE_POS_SIDE_LEN + QR_SIMPLE_POS_2_X - i); col++) {
                *(*(matrix + row) + col) = bit;
            }
        }
        i++;
    }

    /* Let's fill third poisition block */

    i = 0;
    while (i < 3) {
        if (i % 2) {
            bit = QR_SIMPLE_EMPTY_BIT;
        } else {
            bit = QR_SIMPLE_FULL_BIT;
        }

        for (row = QR_SIMPLE_POS_3_Y + i; row < (QR_SIMPLE_POS_SIDE_LEN + QR_SIMPLE_POS_3_Y - i); row++) {
            for (col = QR_SIMPLE_POS_3_X + i; col < (QR_SIMPLE_POS_SIDE_LEN + QR_SIMPLE_POS_3_X - i); col++) {
                *(*(matrix + row) + col) = bit;
            }
        }
        i++;
    }

    /* Let's fill first time block */
    for (col = QR_SIMPLE_TIME_1_X, i = 0; col < (QR_SIMPLE_TIME1_WIDTH + QR_SIMPLE_TIME_1_X); col++, i++) {
        bit = ((i % 2) ? QR_SIMPLE_EMPTY_BIT : QR_SIMPLE_FULL_BIT);
        *(*(matrix + QR_SIMPLE_TIME_1_Y) + col) = bit;
    }

    /* Let's fill second time block */
    for (row = QR_SIMPLE_TIME_2_Y, i = 0; row < (QR_SIMPLE_TIME2_HEIGHT + QR_SIMPLE_TIME_2_Y); row++, i++) {
        bit = ((i % 2) ? QR_SIMPLE_EMPTY_BIT : QR_SIMPLE_FULL_BIT);
        *(*(matrix + row) + QR_SIMPLE_TIME_2_X) = bit;
    }
}

void qr_simple_place_modules(const uint8_t *data_modules, NXWINDOW hwnd)
{
    uint8_t i, j;

    /* Let's initialize array of modules */

    for (i = 0; i< QR_SIMPLE_SIDE_HEIGHT; i++) {
        for (j = 0; j < QR_SIMPLE_SIDE_WIDTH; j++) {
            g_matrix[i][j] = QR_SIMPLE_INIT_BIT;
        }
    }

    qr_simple_place_function_patterns(g_matrix);
    qr_simple_place_data_modules(g_matrix, (uint8_t *)data_modules);
    qr_simple_apply_masks(g_matrix);
    qr_simple_fill_patterns(g_matrix);
    qr_simple_render(g_matrix, hwnd);

#ifdef CONFIG_THINGSEE_DISPLAY_TRACES
    /* Have to use printf's here to display matrix elements without function's name and timestamp */
    for (i = 0; i < QR_SIMPLE_SIDE_HEIGHT; i++) {
        for (j = 0; j < QR_SIMPLE_SIDE_WIDTH; j++) {
            printf("0x%02X ", g_matrix[i][j]);
        }
        printf("\n");
    }
#endif
}
