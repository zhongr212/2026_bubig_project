/**
 * @file bsp_buzzer.h
 * @brief Buzzer control encapsulates implementation.
 * @author Gao Xing
 * @date 2025/2/9
 * @version 1.0
 */

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "tim.h"
#include <stdbool.h>

extern TIM_HandleTypeDef htim4;

#define TIM_CLOCK_FREQ 84000000 ///< 定时器时钟频率为 84 MHz

/**
 * @brief 音符类型枚举
 */
typedef enum
{
    NOTE_DO = 0, ///< Do (C5)
    NOTE_RE,     ///< Re (D5)
    NOTE_MI,     ///< Mi (E5)
    NOTE_FA,     ///< Fa (F5)
    NOTE_SO,     ///< So (G5)
    NOTE_LA,     ///< La (A5)
    NOTE_SI,     ///< Si (B5)
	  NOTE
} Note_Type;

/**
 * @brief 音符结构体，包含音符类型和持续时间
 */
typedef struct
{
    Note_Type note;    // 音符
    uint16_t duration; // 持续时间(ms)
} Music_Note_s;

/**
 * @brief 音乐播放器结构体，用于管理音乐播放状态
 */
typedef struct
{
    const Music_Note_s *notes; // 音符序列
    uint16_t total_notes;      // 总音符数
    uint16_t current_note;     // 当前播放的音符索引
    uint32_t note_start_time;  // 当前音符开始时间
    bool is_playing;           // 是否正在播放
} Music_Player_s;

/**
 * @brief 启动蜂鸣器
 */
void startBuzzer(void);

/**
 * @brief 停止蜂鸣器
 */
void stopBuzzer(void);

/**
 * @brief 播放指定音符
 * @param note 音符类型
 */
void playNote(Note_Type note);

/**
 * @brief 更新音乐播放器状态
 * @param player 音乐播放器指针
 * @details 检查当前音符是否播放完成，需要在主循环中定期调用
 * @note 实现非阻塞式音乐播放的核心函数
 */
void updateMusicPlayer(Music_Player_s *player);

/**
 * @brief 开始播放音乐序列
 * @param player 音乐播放器指针
 * @param notes 音符序列
 * @param total_notes 音符数量
 * @details 初始化播放器状态并开始播放第一个音符
 */
void startMusicPlay(Music_Player_s *player, const Music_Note_s *notes, uint16_t total_notes);

/**
 * @brief 停止音乐播放
 * @param player 音乐播放器指针
 * @details 停止播放并关闭蜂鸣器输出
 */
void stopMusicPlay(Music_Player_s *player);

/** @brief 全局音乐播放器实例 */
extern Music_Player_s music_player;

/**
 * @brief 你愿意和我组一辈子乐队吗
 */
void LowVoltage_Music(void);
void HighHeat_Music(void);
void CAN1_Lost_Music(void);
void CAN2_Lost_Music(void);
void CAN3_Lost_Music(void);
#endif // BSP_BUZZER_H
