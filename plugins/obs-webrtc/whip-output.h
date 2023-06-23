#pragma once
#include <obs-module.h>
#include <obs-avc.h>
#include <obs-nal.h>

#include <util/curl/curl-helper.h>
#include <util/array-serializer.h>
#include <util/platform.h>
#include <util/base.h>
#include <util/dstr.h>

#include <string>
#include <sstream>
#include <atomic>
#include <mutex>
#include <thread>
#include <iostream>
#include <cstdint>
#include <vector>
#include <limits>
#include <random>

#include <rtc/rtc.h>

#define do_log(level, format, ...)                              \
	blog(level, "[obs-webrtc] [whip_output: '%s'] " format, \
	     obs_output_get_name(output), ##__VA_ARGS__)
#define do_log_s(level, format, ...)                            \
	blog(level, "[obs-webrtc] [whip_output: '%s'] " format, \
	     obs_output_get_name(whipOutput->output), ##__VA_ARGS__)

// maximum size for a video fragment, keep it under a std MTU of 1500
// effective range is 576-1470, with the lower the value equating to
// more packets created
const uint16_t max_fragment_size(1180);

class WHIPOutput {
public:
	WHIPOutput(obs_data_t *settings, obs_output_t *output);
	~WHIPOutput();

	bool Start();
	void Stop(bool signal = true);
	void Data(struct encoder_packet *packet);

	inline size_t GetTotalBytes() { return total_bytes_sent; }

	inline int GetConnectTime() { return connect_time_ms; }

private:
	void ConfigureAudioTrack(std::string media_stream_id,
				 std::string cname);
	void ConfigureVideoTrack(std::string media_stream_id,
				 std::string cname);
	bool Init();
	bool Setup();
	bool Connect();
	void StartThread();

	void SendDelete();
	void StopThread(bool signal);

	/**
	* @brief Send data on the specified track.
	* 
	* @param track 
	* @param data 
	* @param size 
	* @param ts elapsed timestamp
	*/
	void Send(int track, void *data, uintptr_t size, uint64_t ts);

	obs_output_t *output;

	std::string endpoint_url;
	std::string bearer_token;
	std::string resource_url;

	std::atomic<bool> running;

	// sprops for h264
	std::string sprop_parameter_sets;

	std::mutex start_stop_mutex;
	std::thread start_stop_thread;

	int peer_connection;

	int audio_track;
	int video_track;

	// total overall bytes sent
	std::atomic<size_t> total_bytes_sent;

	std::atomic<int> connect_time_ms;

	int64_t start_time_ns;
	int64_t last_audio_timestamp;
	int64_t last_video_timestamp;
};

void register_whip_output();

static std::string trim_string(const std::string &source)
{
	std::string ret(source);
	ret.erase(0, ret.find_first_not_of(" \n\r\t"));
	ret.erase(ret.find_last_not_of(" \n\r\t") + 1);
	return ret;
}

static size_t curl_writefunction(char *data, size_t size, size_t nmemb,
				 void *priv_data)
{
	auto read_buffer = static_cast<std::string *>(priv_data);

	size_t real_size = size * nmemb;

	read_buffer->append(data, real_size);
	return real_size;
}

#define LOCATION_HEADER_LENGTH 10

static size_t curl_headerfunction(char *data, size_t size, size_t nmemb,
				  void *priv_data)
{
	auto header_buffer = static_cast<std::string *>(priv_data);

	size_t real_size = size * nmemb;

	if (real_size < LOCATION_HEADER_LENGTH)
		return real_size;

	if (!astrcmpi_n(data, "location: ", LOCATION_HEADER_LENGTH)) {
		char *val = data + LOCATION_HEADER_LENGTH;
		header_buffer->append(val, real_size - LOCATION_HEADER_LENGTH);
		*header_buffer = trim_string(*header_buffer);
	}

	return real_size;
}

/**
 * @brief Generates a random integer for SSRC and starting RTP timestamp.
 * 
 * @return uint32_t 
 */
static uint32_t generate_random_u32()
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<uint32_t> dist(0, UINT32_MAX);
	return dist(gen);
}

/**
 * @brief Parse data of the given length and return any h264 NALU found.
 * 
 * @param data 
 * @param length 
 * @return std::vector<std::vector<uint8_t>> 
 */
static std::vector<std::vector<uint8_t>> parse_h264_nals(const char *data,
							 size_t length)
{
	std::vector<std::vector<uint8_t>> nalus;

	const char *end = data + length;
	const char *start = data;
	const char *current = data;

	while (current < end) {
		// Find the start code prefix (0x000001 or 0x00000001)
		if (current[0] == 0x00 && current[1] == 0x00 &&
		    (current[2] == 0x01 ||
		     (current[2] == 0x00 && current[3] == 0x01))) {
			if (start < current) {
				// Extract the NALU unit
				size_t nalSize = current - start;
				nalus.emplace_back(start, start + nalSize);
			}

			// Move the start pointer to the next NALU
			start = current + 3;
			if (current[2] == 0x00)
				start++;

			// Skip the start code prefix
			current += 3;
			if (current[0] == 0x00)
				current++;
		} else {
			current++;
		}
	}

	// Extract the last NALU unit
	size_t nalSize = current - start;
	if (nalSize > 0)
		nalus.emplace_back(start, start + nalSize);

	return nalus;
}

/*
base64.cpp and base64.h

Copyright (C) 2004-2008 René Nyffenegger

This source code is provided 'as-is', without any express or implied
warranty. In no event will the author be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this source code must not be misrepresented; you must not
claim that you wrote the original source code. If you use this source code
in a product, an acknowledgment in the product documentation would be
appreciated but is not required.

2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original source code.

3. This notice may not be removed or altered from any source distribution.

René Nyffenegger rene.nyffenegger@adp-gmbh.ch

*/

static const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
					"abcdefghijklmnopqrstuvwxyz"
					"0123456789+/";

static std::string b64_encode(const char *bytes_to_encode, size_t in_len)
{
	std::string ret;
	int i = 0;
	int j = 0;
	unsigned char char_array_3[3];
	unsigned char char_array_4[4];

	while (in_len--) {
		char_array_3[i++] = *(bytes_to_encode++);
		if (i == 3) {
			char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
			char_array_4[1] = ((char_array_3[0] & 0x03) << 4) +
					  ((char_array_3[1] & 0xf0) >> 4);
			char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) +
					  ((char_array_3[2] & 0xc0) >> 6);
			char_array_4[3] = char_array_3[2] & 0x3f;

			for (i = 0; (i < 4); i++)
				ret += base64_chars[char_array_4[i]];
			i = 0;
		}
	}

	if (i) {
		for (j = i; j < 3; j++)
			char_array_3[j] = '\0';

		char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
		char_array_4[1] = ((char_array_3[0] & 0x03) << 4) +
				  ((char_array_3[1] & 0xf0) >> 4);
		char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) +
				  ((char_array_3[2] & 0xc0) >> 6);
		char_array_4[3] = char_array_3[2] & 0x3f;

		for (j = 0; (j < i + 1); j++)
			ret += base64_chars[char_array_4[j]];

		while ((i++ < 3))
			ret += '=';
	}

	return ret;
}
