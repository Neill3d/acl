#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2017 Nicholas Frechette & Animation Compression Library contributors
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
////////////////////////////////////////////////////////////////////////////////

#include "acl/core/compiler_utils.h"

#include <cstdint>
#include <cstring>

ACL_IMPL_FILE_PRAGMA_PUSH

namespace acl
{
	// BE CAREFUL WHEN CHANGING VALUES IN THIS ENUM
	// The rotation format is serialized in the compressed data, if you change a value
	// the compressed clips will be invalid. If you do, bump the appropriate algorithm versions.
	enum class RotationFormat8 : uint8_t
	{
		Quat_128					= 0,	// Full precision quaternion, [x,y,z,w] stored with float32
		QuatDropW_96				= 1,	// Full precision quaternion, [x,y,z] stored with float32 (w is dropped)
		QuatDropW_48				= 2,	// Quantized quaternion, [x,y,z] stored with [16,16,16] bits (w is dropped)
		QuatDropW_32				= 3,	// Quantized quaternion, [x,y,z] stored with [11,11,10] bits (w is dropped)
		QuatDropW_Variable			= 4,	// Quantized quaternion, [x,y,z] stored with [N,N,N] bits (w is dropped, same number of bits per component)
		// TODO: Implement these
		//Quat_32_Largest,		// Quantized quaternion, [?,?,?] stored with [10,10,10] bits (largest component is dropped, component index stored on 2 bits)
		//QuatLog_96,			// Full precision quaternion logarithm, [x,y,z] stored with float 32
		//QuatLog_48,			// Quantized quaternion logarithm, [x,y,z] stored with [16,16,16] bits
		//QuatLog_32,			// Quantized quaternion logarithm, [x,y,z] stored with [11,11,10] bits
		//QuatLog_Variable,		// Quantized quaternion logarithm, [x,y,z] stored with [N,N,N] bits (same number of bits per component)
	};

	// BE CAREFUL WHEN CHANGING VALUES IN THIS ENUM
	// The vector format is serialized in the compressed data, if you change a value
	// the compressed clips will be invalid. If you do, bump the appropriate algorithm versions.
	enum class VectorFormat8 : uint8_t
	{
		Vector3_96					= 0,	// Full precision vector3, [x,y,z] stored with float32
		Vector3_48					= 1,	// Quantized vector3, [x,y,z] stored with [16,16,16] bits
		Vector3_32					= 2,	// Quantized vector3, [x,y,z] stored with [11,11,10] bits
		Vector3_Variable			= 3,	// Quantized vector3, [x,y,z] stored with [N,N,N] bits (same number of bits per component)
	};

	union TrackFormat8
	{
		RotationFormat8 rotation;
		VectorFormat8 vector;

		TrackFormat8() {}
		TrackFormat8(RotationFormat8 format) : rotation(format) {}
		TrackFormat8(VectorFormat8 format) : vector(format) {}
	};

	enum class AnimationTrackType8 : uint8_t
	{
		Rotation,
		Translation,
		Scale,
	};

	enum class RotationVariant8 : uint8_t
	{
		Quat,
		QuatDropW,
		//QuatDropLargest,
		//QuatLog,
	};

	//ACL_DEPRECATED("No longer used by decompression functions, to be removed in v2.0")
	enum class TimeSeriesType8 : uint8_t
	{
		Constant,
		ConstantDefault,
		Varying,
	};

	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	// We only support up to 4294967295 tracks. We reserve 4294967295 for the invalid index
	constexpr uint32_t k_invalid_track_index = 0xFFFFFFFFu;

	enum class track_type8 : uint8_t
	{
		float1f		= 0,
		float2f		= 1,
		float3f		= 2,
		float4f		= 3,
		vector4f	= 4,

		//float1d	= 5,
		//float2d	= 6,
		//float3d	= 7,
		//float4d	= 8,
		//vector4d	= 9,

		//quatf		= 10,
		//quatd		= 11,

		//qvvf		= 12,
		//qvvd		= 13,

		//int1i		= 14,
		//int2i		= 15,
		//int3i		= 16,
		//int4i		= 17,
		//vector4i	= 18,

		//int1q		= 19,
		//int2q		= 20,
		//int3q		= 21,
		//int4q		= 22,
		//vector4q	= 23,
	};

	enum class track_category8 : uint8_t
	{
		scalarf		= 0,
		//scalard	= 1,
		//scalari	= 2,
		//scalarq	= 3,
		//transformf = 4,
		//transformd = 5,
	};

	// Used by: float1f, float2f, float3f, float4f
	struct track_desc_scalarf
	{
		static constexpr track_category8 category = track_category8::scalarf;

		// The track output index. When writing out the compressed data stream, this index
		// will be used instead of the track index. This allows custom reordering for things
		// like LOD sorting or skeleton remapping. A value of 'k_invalid_track_index' will strip the track
		// from the compressed data stream. Output indices must be unique and contiguous.
		uint32_t output_index;

		float precision;

		// TODO: Use the precision?
		float constant_threshold;
	};

#if 0	// TODO: Add support for this
	// Used by: quatf, qvvf
	struct track_desc_transformf
	{
		static constexpr track_category8 category = track_category8::transformf;

		// The track output index. When writing out the compressed data stream, this index
		// will be used instead of the track index. This allows custom reordering for things
		// like LOD sorting or skeleton remapping. A value of 'k_invalid_track_index' will strip the track
		// from the compressed data stream. Output indices must be unique and contiguous.
		uint32_t output_index;

		uint32_t parent_index;

		float precision;

		// TODO: Use the precision and shell distance?
		float constant_rotation_threshold;
		float constant_translation_threshold;
		float constant_scale_threshold;

		float shell_distance;
	};
#endif

	// TODO: Add transform description?

	//////////////////////////////////////////////////////////////////////////

	// Bit rate 0 is reserved for tracks that are constant in a segment
	constexpr uint8_t k_bit_rate_num_bits[] = { 0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 32 };

	constexpr uint8_t k_invalid_bit_rate = 0xFF;
	constexpr uint8_t k_lowest_bit_rate = 1;
	constexpr uint8_t k_highest_bit_rate = sizeof(k_bit_rate_num_bits) - 1;
	constexpr uint8_t k_num_bit_rates = sizeof(k_bit_rate_num_bits);

	static_assert(k_num_bit_rates == 19, "Expecting 19 bit rates");

	// If all tracks are variable, no need for any extra padding except at the very end of the data
	// If our tracks are mixed variable/not variable, we need to add some padding to ensure alignment
	constexpr uint8_t k_mixed_packing_alignment_num_bits = 16;

	inline uint8_t get_num_bits_at_bit_rate(uint8_t bit_rate)
	{
		ACL_ASSERT(bit_rate <= k_highest_bit_rate, "Invalid bit rate: %u", bit_rate);
		return k_bit_rate_num_bits[bit_rate];
	}

	// Track is constant, our constant sample is stored in the range information
	constexpr bool is_constant_bit_rate(uint8_t bit_rate) { return bit_rate == 0; }
	constexpr bool is_raw_bit_rate(uint8_t bit_rate) { return bit_rate == k_highest_bit_rate; }

	struct BoneBitRate
	{
		uint8_t rotation;
		uint8_t translation;
		uint8_t scale;
	};

	//////////////////////////////////////////////////////////////////////////

	// TODO: constexpr
	inline const char* get_rotation_format_name(RotationFormat8 format)
	{
		switch (format)
		{
		case RotationFormat8::Quat_128:				return "Quat_128";
		case RotationFormat8::QuatDropW_96:			return "QuatDropW_96";
		case RotationFormat8::QuatDropW_48:			return "QuatDropW_48";
		case RotationFormat8::QuatDropW_32:			return "QuatDropW_32";
		case RotationFormat8::QuatDropW_Variable:	return "QuatDropW_Variable";
		default:									return "<Invalid>";
		}
	}

	inline bool get_rotation_format(const char* format, RotationFormat8& out_format)
	{
		const char* quat_128_format = "Quat_128";
		if (std::strncmp(format, quat_128_format, std::strlen(quat_128_format)) == 0)
		{
			out_format = RotationFormat8::Quat_128;
			return true;
		}

		const char* quatdropw_96_format = "QuatDropW_96";
		if (std::strncmp(format, quatdropw_96_format, std::strlen(quatdropw_96_format)) == 0)
		{
			out_format = RotationFormat8::QuatDropW_96;
			return true;
		}

		const char* quatdropw_48_format = "QuatDropW_48";
		if (std::strncmp(format, quatdropw_48_format, std::strlen(quatdropw_48_format)) == 0)
		{
			out_format = RotationFormat8::QuatDropW_48;
			return true;
		}

		const char* quatdropw_32_format = "QuatDropW_32";
		if (std::strncmp(format, quatdropw_32_format, std::strlen(quatdropw_32_format)) == 0)
		{
			out_format = RotationFormat8::QuatDropW_32;
			return true;
		}

		const char* quatdropw_variable_format = "QuatDropW_Variable";
		if (std::strncmp(format, quatdropw_variable_format, std::strlen(quatdropw_variable_format)) == 0)
		{
			out_format = RotationFormat8::QuatDropW_Variable;
			return true;
		}

		return false;
	}

	// TODO: constexpr
	inline const char* get_vector_format_name(VectorFormat8 format)
	{
		switch (format)
		{
		case VectorFormat8::Vector3_96:			return "Vector3_96";
		case VectorFormat8::Vector3_48:			return "Vector3_48";
		case VectorFormat8::Vector3_32:			return "Vector3_32";
		case VectorFormat8::Vector3_Variable:	return "Vector3_Variable";
		default:								return "<Invalid>";
		}
	}

	inline bool get_vector_format(const char* format, VectorFormat8& out_format)
	{
		const char* vector3_96_format = "Vector3_96";
		if (std::strncmp(format, vector3_96_format, std::strlen(vector3_96_format)) == 0)
		{
			out_format = VectorFormat8::Vector3_96;
			return true;
		}

		const char* vector3_48_format = "Vector3_48";
		if (std::strncmp(format, vector3_48_format, std::strlen(vector3_48_format)) == 0)
		{
			out_format = VectorFormat8::Vector3_48;
			return true;
		}

		const char* vector3_32_format = "Vector3_32";
		if (std::strncmp(format, vector3_32_format, std::strlen(vector3_32_format)) == 0)
		{
			out_format = VectorFormat8::Vector3_32;
			return true;
		}

		const char* vector3_variable_format = "Vector3_Variable";
		if (std::strncmp(format, vector3_variable_format, std::strlen(vector3_variable_format)) == 0)
		{
			out_format = VectorFormat8::Vector3_Variable;
			return true;
		}

		return false;
	}

	// TODO: constexpr
	inline RotationVariant8 get_rotation_variant(RotationFormat8 rotation_format)
	{
		switch (rotation_format)
		{
		case RotationFormat8::Quat_128:
			return RotationVariant8::Quat;
		case RotationFormat8::QuatDropW_96:
		case RotationFormat8::QuatDropW_48:
		case RotationFormat8::QuatDropW_32:
		case RotationFormat8::QuatDropW_Variable:
			return RotationVariant8::QuatDropW;
		default:
			ACL_ASSERT(false, "Invalid or unsupported rotation format: %s", get_rotation_format_name(rotation_format));
			return RotationVariant8::Quat;
		}
	}

	// TODO: constexpr
	inline RotationFormat8 get_lowest_variant_precision(RotationVariant8 variant)
	{
		switch (variant)
		{
		case RotationVariant8::Quat:			return RotationFormat8::Quat_128;
		case RotationVariant8::QuatDropW:		return RotationFormat8::QuatDropW_32;
		default:
			ACL_ASSERT(false, "Invalid or unsupported rotation format: %u", (uint32_t)variant);
			return RotationFormat8::Quat_128;
		}
	}

	// TODO: constexpr
	inline RotationFormat8 get_highest_variant_precision(RotationVariant8 variant)
	{
		switch (variant)
		{
		case RotationVariant8::Quat:			return RotationFormat8::Quat_128;
		case RotationVariant8::QuatDropW:		return RotationFormat8::QuatDropW_96;
		default:
			ACL_ASSERT(false, "Invalid or unsupported rotation format: %u", (uint32_t)variant);
			return RotationFormat8::Quat_128;
		}
	}

	// TODO: constexpr
	inline bool is_rotation_format_variable(RotationFormat8 rotation_format)
	{
		switch (rotation_format)
		{
		case RotationFormat8::Quat_128:
		case RotationFormat8::QuatDropW_96:
		case RotationFormat8::QuatDropW_48:
		case RotationFormat8::QuatDropW_32:
			return false;
		case RotationFormat8::QuatDropW_Variable:
			return true;
		default:
			ACL_ASSERT(false, "Invalid or unsupported rotation format: %s", get_rotation_format_name(rotation_format));
			return false;
		}
	}

	constexpr bool is_vector_format_variable(VectorFormat8 format)
	{
		return format == VectorFormat8::Vector3_Variable;
	}

	// TODO: constexpr
	inline const char* get_track_type_name(track_type8 type)
	{
		switch (type)
		{
		case track_type8::float1f:			return "float1f";
		case track_type8::float2f:			return "float2f";
		case track_type8::float3f:			return "float3f";
		case track_type8::float4f:			return "float4f";
		case track_type8::vector4f:			return "vector4f";
		default:							return "<Invalid>";
		}
	}

	inline bool get_track_type(const char* type, track_type8& out_type)
	{
		const char* float1f_type = "float1f";
		if (std::strncmp(type, float1f_type, std::strlen(float1f_type)) == 0)
		{
			out_type = track_type8::float1f;
			return true;
		}

		const char* float2f_type = "float2f";
		if (std::strncmp(type, float2f_type, std::strlen(float2f_type)) == 0)
		{
			out_type = track_type8::float2f;
			return true;
		}

		const char* float3f_type = "float3f";
		if (std::strncmp(type, float3f_type, std::strlen(float3f_type)) == 0)
		{
			out_type = track_type8::float3f;
			return true;
		}

		const char* float4f_type = "float4f";
		if (std::strncmp(type, float4f_type, std::strlen(float4f_type)) == 0)
		{
			out_type = track_type8::float4f;
			return true;
		}

		const char* vector4f_type = "vector4f";
		if (std::strncmp(type, vector4f_type, std::strlen(vector4f_type)) == 0)
		{
			out_type = track_type8::vector4f;
			return true;
		}

		return false;
	}

	// TODO: constexpr
	inline track_category8 get_track_category(track_type8 type)
	{
		switch (type)
		{
		default:
		case track_type8::float1f:		return track_category8::scalarf;
		case track_type8::float2f:		return track_category8::scalarf;
		case track_type8::float3f:		return track_category8::scalarf;
		case track_type8::float4f:		return track_category8::scalarf;
		case track_type8::vector4f:		return track_category8::scalarf;
		}
	}

	// TODO: constexpr
	inline uint32_t get_track_num_sample_elements(track_type8 type)
	{
		switch (type)
		{
		case track_type8::float1f:		return 1;
		case track_type8::float2f:		return 2;
		case track_type8::float3f:		return 3;
		case track_type8::float4f:		return 4;
		case track_type8::vector4f:		return 4;
		default:
			ACL_ASSERT(false, "Unsupported track type");
			return 0;
		}
	}
}

ACL_IMPL_FILE_PRAGMA_POP
