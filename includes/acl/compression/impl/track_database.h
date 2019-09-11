#pragma once

////////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Copyright (c) 2019 Nicholas Frechette & Animation Compression Library contributors
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
#include "acl/core/iallocator.h"
#include "acl/compression/animation_clip.h"
#include "acl/compression/compression_settings.h"
#include "acl/compression/skeleton.h"
#include "acl/compression/stream/segment_context.h"

#include <cstdint>

// 0 = disabled, 1 = enabled
#define ACL_IMPL_DEBUG_DATABASE_IMPL 0

ACL_IMPL_FILE_PRAGMA_PUSH

namespace acl
{
	namespace acl_impl
	{
		static constexpr uint32_t k_simd_width			= 4;		// NEON/SSE width for now
		static constexpr uint32_t k_simd_padding_width	= 8;		// Twice the width to support interleaving two iterations when possible

		constexpr uint32_t get_num_components_per_transform(bool has_scale)
		{
			return has_scale ? 7 : 10;	// rot(4) + trans(3) + optional scale(3)
		}

		//////////////////////////////////////////////////////////////////////////
		// A track database manages a contiguous buffer of all the track data.
		// This buffer is mutable and stored in SOA form:
		// sample0.x, sample1.x, sample2.x, sample3.x, ..., sample0.y, sample1.y, ...
		// Memory layout:
		//    track 0
		//        rotation: sample0.x, sample1.x, sample2.x, sample3.x, ..., sample0.y, sample1.y, sample2.y, sample3.y, ...
		//        translation: sample0.x, sample1.x, sample2.x, sample3.x, ..., sample0.y, sample1.y, sample2.y, sample3.y, ...
		//        scale (optional):  sample0.x, sample1.x, sample2.x, sample3.x, ..., sample0.y, sample1.y, sample2.y, sample3.y, ...
		//    track 1
		// ...
		// We round up the number of samples for each segment to a multiple of the SIMD width (e.g. 4) to avoid partial loops.
		// Each component takes: sizeof(float) * num_simd_samples
		// Rotations have 4 components, translation and scale have 3 each
		// Each transform has the size: component_size * (4 + 3 + 3)
		// Each segment has the size: transform_size * num_transforms
		// Each component and transform has a fixed size per segment but each segment can have a different size
		// and as such we store the segment_data_start_offset in each segment as well as its size.
		// We can offset into our contiguous SOA buffer to the current transform with: transform_index * transform_size
		// Our individual component start offsets with: component_index * component_size
		//////////////////////////////////////////////////////////////////////////
		class track_database
		{
		public:
			inline track_database(IAllocator& allocator, const AnimationClip& clip, const RigidSkeleton& skeleton, const CompressionSettings& settings, const segment_context* segments, uint32_t num_segments);
			inline ~track_database();

			inline uint32_t get_num_transforms() const { return m_num_transforms; }
			inline uint32_t get_num_samples_per_track() const { return m_num_samples_per_track; }
			inline bool has_scale() const { return m_has_scale; }
			inline Vector4_32 ACL_SIMD_CALL get_default_scale() const { return m_default_scale; }
			inline float get_sample_rate() const { return m_sample_rate; }
			inline float get_duration() const { return m_duration; }

			inline RotationFormat8 get_rotation_format() const { return m_rotation_format; }
			inline void set_rotation_format(RotationFormat8 format) { m_rotation_format = format; }
			inline VectorFormat8 get_translation_format() const { return m_translation_format; }
			inline VectorFormat8 get_scale_format() const { return m_scale_format; }

			inline qvvf_ranges& get_range(uint32_t transform_index) { return m_ranges[transform_index]; }
			inline const qvvf_ranges& get_range(uint32_t transform_index) const { return m_ranges[transform_index]; }

			inline uint32_t get_parent_index(uint32_t transform_index) const { return m_skeleton.get_bones()[transform_index].parent_index; }

			inline void get_rotations(const segment_context& segment, uint32_t transform_index, Vector4_32*& out_rotations_x, Vector4_32*& out_rotations_y, Vector4_32*& out_rotations_z, Vector4_32*& out_rotations_w);
			inline void get_translations(const segment_context& segment, uint32_t transform_index, Vector4_32*& out_translations_x, Vector4_32*& out_translations_y, Vector4_32*& out_translations_z);
			inline void get_scales(const segment_context& segment, uint32_t transform_index, Vector4_32*& out_scales_x, Vector4_32*& out_scales_y, Vector4_32*& out_scales_z);

			inline Vector4_32 ACL_SIMD_CALL get_rotation(const segment_context& segment, uint32_t transform_index, uint32_t sample_index) const;
			inline Vector4_32 ACL_SIMD_CALL get_translation(const segment_context& segment, uint32_t transform_index, uint32_t sample_index) const;
			inline Vector4_32 ACL_SIMD_CALL get_scale(const segment_context& segment, uint32_t transform_index, uint32_t sample_index) const;

			inline void ACL_SIMD_CALL set_rotation(Quat_32Arg0 rotation, const segment_context& segment, uint32_t transform_index, uint32_t sample_index);
			inline void ACL_SIMD_CALL set_translation(Vector4_32Arg0 translation, const segment_context& segment, uint32_t transform_index, uint32_t sample_index);
			inline void ACL_SIMD_CALL set_scale(Vector4_32Arg0 scale, const segment_context& segment, uint32_t transform_index, uint32_t sample_index);

#if 0
			inline uint32_t get_rotation_sample_num_bits(uint32_t transform_index) const;
			inline uint32_t get_translation_sample_num_bits(uint32_t transform_index) const;
			inline uint32_t get_scale_sample_num_bits(uint32_t transform_index) const;
#endif

		private:
			Vector4_32			m_default_scale;

			IAllocator&				m_allocator;
			const RigidSkeleton&	m_skeleton;

			uint32_t			m_num_transforms;
			uint32_t			m_num_tracks;
			uint32_t			m_num_samples_per_track;

			float				m_sample_rate;
			float				m_duration;

			RotationFormat8		m_rotation_format;
			VectorFormat8		m_translation_format;
			VectorFormat8		m_scale_format;

			bool				m_has_scale;

			qvvf_ranges*		m_ranges;

			uint8_t*			m_data;
			size_t				m_data_size;
		};

		//////////////////////////////////////////////////////////////////////////
		// Implementation

		inline track_database::track_database(IAllocator& allocator, const AnimationClip& clip, const RigidSkeleton& skeleton, const CompressionSettings& settings, const segment_context* segments, uint32_t num_segments)
			: m_allocator(allocator)
			, m_skeleton(skeleton)
			, m_sample_rate(clip.get_sample_rate())
			, m_duration(clip.get_duration())
			, m_rotation_format(RotationFormat8::Quat_128)
			, m_translation_format(VectorFormat8::Vector3_96)
			, m_scale_format(VectorFormat8::Vector3_96)
		{
			const bool has_scale_ = clip.has_scale(settings.constant_scale_threshold);
			m_has_scale = has_scale_;

			const uint32_t num_samples_per_track = clip.get_num_samples();
			m_num_samples_per_track = num_samples_per_track;

			const uint16_t num_transforms = clip.get_num_bones();
			m_num_transforms = num_transforms;

			const uint32_t num_tracks_per_transform = has_scale_ ? 3 : 2;
			const uint32_t num_components_per_transform = get_num_components_per_transform(has_scale_);

			const uint32_t num_tracks = num_tracks_per_transform * num_transforms;
			m_num_tracks = num_tracks;

			m_ranges = allocate_type_array<qvvf_ranges>(allocator, num_transforms);
			std::memset(m_ranges, 0, sizeof(qvvf_ranges) * num_transforms);

			uint32_t data_size = 0;
			for (uint32_t segment_index = 0; segment_index < num_segments; ++segment_index)
				data_size += segments[segment_index].soa_size;

			m_data = reinterpret_cast<uint8_t*>(allocator.allocate(data_size, 64));
			m_data_size = data_size;

			m_default_scale = acl::get_default_scale(clip.get_additive_format());

			// Copy the data into our database
			for (uint32_t segment_index = 0; segment_index < num_segments; ++segment_index)
			{
				const segment_context& segment = segments[segment_index];

				const uint32_t num_simd_samples_per_track = segment.num_simd_samples_per_track;
				const uint32_t component_size = sizeof(float) * num_simd_samples_per_track;
				const uint32_t transform_size = component_size * num_components_per_transform;

				const uint32_t rotation_track_size = component_size * 4;
				const uint32_t translation_track_size = component_size * 3;

				const uint32_t offset_x = 0;
				const uint32_t offset_y = offset_x + component_size;
				const uint32_t offset_z = offset_y + component_size;
				const uint32_t offset_w = offset_z + component_size;

				uint8_t* segment_data = m_data + segment.soa_start_offset;

				for (uint16_t transform_index = 0; transform_index < num_transforms; ++transform_index)
				{
					const AnimatedBone& transform = clip.get_animated_bone(transform_index);

					const uint32_t transform_offset = transform_index * transform_size;

					const uint32_t rotation_offset = transform_offset;
					const uint32_t translation_offset = rotation_offset + rotation_track_size;
					const uint32_t scale_offset = translation_offset + translation_track_size;

					float* rotation_track_x = safe_ptr_cast<float>(segment_data + rotation_offset + offset_x);
					float* rotation_track_y = safe_ptr_cast<float>(segment_data + rotation_offset + offset_y);
					float* rotation_track_z = safe_ptr_cast<float>(segment_data + rotation_offset + offset_z);
					float* rotation_track_w = safe_ptr_cast<float>(segment_data + rotation_offset + offset_w);

					float* translation_track_x = safe_ptr_cast<float>(segment_data + translation_offset + offset_x);
					float* translation_track_y = safe_ptr_cast<float>(segment_data + translation_offset + offset_y);
					float* translation_track_z = safe_ptr_cast<float>(segment_data + translation_offset + offset_z);

					float* scale_track_x = safe_ptr_cast<float>(segment_data + scale_offset + offset_x);
					float* scale_track_y = safe_ptr_cast<float>(segment_data + scale_offset + offset_y);
					float* scale_track_z = safe_ptr_cast<float>(segment_data + scale_offset + offset_z);

					// Copy the samples into SOA layout
					for (uint32_t sample_index = 0; sample_index < segment.num_samples_per_track; ++sample_index)
					{
						const Quat_32 rotation = quat_normalize(quat_cast(transform.rotation_track.get_sample(sample_index)));
						rotation_track_x[sample_index] = quat_get_x(rotation);
						rotation_track_y[sample_index] = quat_get_y(rotation);
						rotation_track_z[sample_index] = quat_get_z(rotation);
						rotation_track_w[sample_index] = quat_get_w(rotation);

						const Vector4_32 translation = vector_cast(transform.translation_track.get_sample(sample_index));
						translation_track_x[sample_index] = vector_get_x(translation);
						translation_track_y[sample_index] = vector_get_y(translation);
						translation_track_z[sample_index] = vector_get_z(translation);

						if (has_scale_)
						{
							const Vector4_32 scale = vector_cast(transform.scale_track.get_sample(sample_index));
							scale_track_x[sample_index] = vector_get_x(scale);
							scale_track_y[sample_index] = vector_get_y(scale);
							scale_track_z[sample_index] = vector_get_z(scale);
						}
					}

					// Add padding by repeating the last sample
					for (uint32_t sample_index = segment.num_samples_per_track; sample_index < num_simd_samples_per_track; ++sample_index)
					{
						// TODO: write the default value to a float array to avoid potentially shuffling everytime
						rotation_track_x[sample_index] = rotation_track_x[segment.num_samples_per_track - 1];
						rotation_track_y[sample_index] = rotation_track_y[segment.num_samples_per_track - 1];
						rotation_track_z[sample_index] = rotation_track_z[segment.num_samples_per_track - 1];
						rotation_track_w[sample_index] = rotation_track_w[segment.num_samples_per_track - 1];

						translation_track_x[sample_index] = translation_track_x[segment.num_samples_per_track - 1];
						translation_track_y[sample_index] = translation_track_y[segment.num_samples_per_track - 1];
						translation_track_z[sample_index] = translation_track_z[segment.num_samples_per_track - 1];

						if (has_scale_)
						{
							scale_track_x[sample_index] = scale_track_x[segment.num_samples_per_track - 1];
							scale_track_y[sample_index] = scale_track_y[segment.num_samples_per_track - 1];
							scale_track_z[sample_index] = scale_track_z[segment.num_samples_per_track - 1];
						}
					}
				}
			}
		}

		inline track_database::~track_database()
		{
			deallocate_type_array(m_allocator, m_ranges, m_num_transforms);
			m_allocator.deallocate(m_data, m_data_size);
		}

		inline void track_database::get_rotations(const segment_context& segment, uint32_t transform_index, Vector4_32*& out_rotations_x, Vector4_32*& out_rotations_y, Vector4_32*& out_rotations_z, Vector4_32*& out_rotations_w)
		{
			const uint32_t num_components_per_transform = get_num_components_per_transform(m_has_scale);
			const uint32_t num_simd_samples_per_track = segment.num_simd_samples_per_track;
			const uint32_t component_size = sizeof(float) * num_simd_samples_per_track;
			const uint32_t transform_size = component_size * num_components_per_transform;

			const uint32_t offset_x = 0;
			const uint32_t offset_y = offset_x + component_size;
			const uint32_t offset_z = offset_y + component_size;
			const uint32_t offset_w = offset_z + component_size;

			const uint32_t transform_offset = transform_index * transform_size;

			const uint32_t rotation_offset = transform_offset;

			uint8_t* segment_data = m_data + segment.soa_start_offset;
			uint8_t* rotation_data = segment_data + rotation_offset;
			out_rotations_x = safe_ptr_cast<Vector4_32>(rotation_data + offset_x);
			out_rotations_y = safe_ptr_cast<Vector4_32>(rotation_data + offset_y);
			out_rotations_z = safe_ptr_cast<Vector4_32>(rotation_data + offset_z);
			out_rotations_w = safe_ptr_cast<Vector4_32>(rotation_data + offset_w);
		}

		inline void track_database::get_translations(const segment_context& segment, uint32_t transform_index, Vector4_32*& out_translations_x, Vector4_32*& out_translations_y, Vector4_32*& out_translations_z)
		{
			const uint32_t num_components_per_transform = get_num_components_per_transform(m_has_scale);
			const uint32_t num_simd_samples_per_track = segment.num_simd_samples_per_track;
			const uint32_t component_size = sizeof(float) * num_simd_samples_per_track;
			const uint32_t transform_size = component_size * num_components_per_transform;

			const uint32_t rotation_track_size = component_size * 4;

			const uint32_t offset_x = 0;
			const uint32_t offset_y = offset_x + component_size;
			const uint32_t offset_z = offset_y + component_size;

			const uint32_t transform_offset = transform_index * transform_size;

			const uint32_t rotation_offset = transform_offset;
			const uint32_t translation_offset = rotation_offset + rotation_track_size;

			uint8_t* segment_data = m_data + segment.soa_start_offset;
			uint8_t* translation_data = segment_data + translation_offset;
			out_translations_x = safe_ptr_cast<Vector4_32>(translation_data + offset_x);
			out_translations_y = safe_ptr_cast<Vector4_32>(translation_data + offset_y);
			out_translations_z = safe_ptr_cast<Vector4_32>(translation_data + offset_z);
		}

		inline void track_database::get_scales(const segment_context& segment, uint32_t transform_index, Vector4_32*& out_scales_x, Vector4_32*& out_scales_y, Vector4_32*& out_scales_z)
		{
			if (!m_has_scale)
			{
				out_scales_x = nullptr;
				out_scales_y = nullptr;
				out_scales_z = nullptr;
				return;
			}

			const uint32_t num_components_per_transform = get_num_components_per_transform(m_has_scale);
			const uint32_t num_simd_samples_per_track = segment.num_simd_samples_per_track;
			const uint32_t component_size = sizeof(float) * num_simd_samples_per_track;
			const uint32_t transform_size = component_size * num_components_per_transform;

			const uint32_t rotation_track_size = component_size * 4;
			const uint32_t translation_track_size = component_size * 3;

			const uint32_t offset_x = 0;
			const uint32_t offset_y = offset_x + component_size;
			const uint32_t offset_z = offset_y + component_size;

			const uint32_t transform_offset = transform_index * transform_size;

			const uint32_t rotation_offset = transform_offset;
			const uint32_t translation_offset = rotation_offset + rotation_track_size;
			const uint32_t scale_offset = translation_offset + translation_track_size;

			uint8_t* segment_data = m_data + segment.soa_start_offset;
			uint8_t* scale_data = segment_data + scale_offset;
			out_scales_x = safe_ptr_cast<Vector4_32>(scale_data + offset_x);
			out_scales_y = safe_ptr_cast<Vector4_32>(scale_data + offset_y);
			out_scales_z = safe_ptr_cast<Vector4_32>(scale_data + offset_z);
		}

		inline Vector4_32 ACL_SIMD_CALL track_database::get_rotation(const segment_context& segment, uint32_t transform_index, uint32_t sample_index) const
		{
			const uint32_t num_components_per_transform = get_num_components_per_transform(m_has_scale);
			const uint32_t num_simd_samples_per_track = segment.num_simd_samples_per_track;
			const uint32_t component_size = sizeof(float) * num_simd_samples_per_track;
			const uint32_t transform_size = component_size * num_components_per_transform;

			const uint32_t offset_x = 0;
			const uint32_t offset_y = offset_x + component_size;
			const uint32_t offset_z = offset_y + component_size;
			const uint32_t offset_w = offset_z + component_size;

			const uint32_t transform_offset = transform_index * transform_size;

			const uint32_t rotation_offset = transform_offset;

			const uint8_t* segment_data = m_data + segment.soa_start_offset;
			const uint8_t* rotation_data = segment_data + rotation_offset;
			const float* rotations_x = safe_ptr_cast<const float>(rotation_data + offset_x);
			const float* rotations_y = safe_ptr_cast<const float>(rotation_data + offset_y);
			const float* rotations_z = safe_ptr_cast<const float>(rotation_data + offset_z);
			const float* rotations_w = safe_ptr_cast<const float>(rotation_data + offset_w);

			return vector_set(rotations_x[sample_index], rotations_y[sample_index], rotations_z[sample_index], rotations_w[sample_index]);
		}

		inline Vector4_32 ACL_SIMD_CALL track_database::get_translation(const segment_context& segment, uint32_t transform_index, uint32_t sample_index) const
		{
			const uint32_t num_components_per_transform = get_num_components_per_transform(m_has_scale);
			const uint32_t num_simd_samples_per_track = segment.num_simd_samples_per_track;
			const uint32_t component_size = sizeof(float) * num_simd_samples_per_track;
			const uint32_t transform_size = component_size * num_components_per_transform;

			const uint32_t rotation_track_size = component_size * 4;

			const uint32_t offset_x = 0;
			const uint32_t offset_y = offset_x + component_size;
			const uint32_t offset_z = offset_y + component_size;

			const uint32_t transform_offset = transform_index * transform_size;

			const uint32_t rotation_offset = transform_offset;
			const uint32_t translation_offset = rotation_offset + rotation_track_size;

			const uint8_t* segment_data = m_data + segment.soa_start_offset;
			const uint8_t* translation_data = segment_data + translation_offset;
			const float* translations_x = safe_ptr_cast<const float>(translation_data + offset_x);
			const float* translations_y = safe_ptr_cast<const float>(translation_data + offset_y);
			const float* translations_z = safe_ptr_cast<const float>(translation_data + offset_z);

			return vector_set(translations_x[sample_index], translations_y[sample_index], translations_z[sample_index], 0.0f);
		}

		inline Vector4_32 ACL_SIMD_CALL track_database::get_scale(const segment_context& segment, uint32_t transform_index, uint32_t sample_index) const
		{
			if (!m_has_scale)
				return m_default_scale;

			const uint32_t num_components_per_transform = get_num_components_per_transform(m_has_scale);
			const uint32_t num_simd_samples_per_track = segment.num_simd_samples_per_track;
			const uint32_t component_size = sizeof(float) * num_simd_samples_per_track;
			const uint32_t transform_size = component_size * num_components_per_transform;

			const uint32_t rotation_track_size = component_size * 4;
			const uint32_t translation_track_size = component_size * 3;

			const uint32_t offset_x = 0;
			const uint32_t offset_y = offset_x + component_size;
			const uint32_t offset_z = offset_y + component_size;

			const uint32_t transform_offset = transform_index * transform_size;

			const uint32_t rotation_offset = transform_offset;
			const uint32_t translation_offset = rotation_offset + rotation_track_size;
			const uint32_t scale_offset = translation_offset + translation_track_size;

			const uint8_t* segment_data = m_data + segment.soa_start_offset;
			const uint8_t* scale_data = segment_data + scale_offset;
			const float* scales_x = safe_ptr_cast<const float>(scale_data + offset_x);
			const float* scales_y = safe_ptr_cast<const float>(scale_data + offset_y);
			const float* scales_z = safe_ptr_cast<const float>(scale_data + offset_z);

			return vector_set(scales_x[sample_index], scales_y[sample_index], scales_z[sample_index], 0.0f);
		}

		inline void ACL_SIMD_CALL track_database::set_rotation(Quat_32Arg0 rotation, const segment_context& segment, uint32_t transform_index, uint32_t sample_index)
		{
			const uint32_t num_components_per_transform = get_num_components_per_transform(m_has_scale);
			const uint32_t num_simd_samples_per_track = segment.num_simd_samples_per_track;
			const uint32_t component_size = sizeof(float) * num_simd_samples_per_track;
			const uint32_t transform_size = component_size * num_components_per_transform;

			const uint32_t offset_x = 0;
			const uint32_t offset_y = offset_x + component_size;
			const uint32_t offset_z = offset_y + component_size;
			const uint32_t offset_w = offset_z + component_size;

			const uint32_t transform_offset = transform_index * transform_size;

			const uint32_t rotation_offset = transform_offset;

			uint8_t* segment_data = m_data + segment.soa_start_offset;
			uint8_t* rotation_data = segment_data + rotation_offset;
			float* rotations_x = safe_ptr_cast<float>(rotation_data + offset_x);
			float* rotations_y = safe_ptr_cast<float>(rotation_data + offset_y);
			float* rotations_z = safe_ptr_cast<float>(rotation_data + offset_z);
			float* rotations_w = safe_ptr_cast<float>(rotation_data + offset_w);

			rotations_x[sample_index] = quat_get_x(rotation);
			rotations_y[sample_index] = quat_get_y(rotation);
			rotations_z[sample_index] = quat_get_z(rotation);
			rotations_w[sample_index] = quat_get_w(rotation);
		}

		inline void ACL_SIMD_CALL track_database::set_translation(Vector4_32Arg0 translation, const segment_context& segment, uint32_t transform_index, uint32_t sample_index)
		{
			const uint32_t num_components_per_transform = get_num_components_per_transform(m_has_scale);
			const uint32_t num_simd_samples_per_track = segment.num_simd_samples_per_track;
			const uint32_t component_size = sizeof(float) * num_simd_samples_per_track;
			const uint32_t transform_size = component_size * num_components_per_transform;

			const uint32_t rotation_track_size = component_size * 4;

			const uint32_t offset_x = 0;
			const uint32_t offset_y = offset_x + component_size;
			const uint32_t offset_z = offset_y + component_size;

			const uint32_t transform_offset = transform_index * transform_size;

			const uint32_t rotation_offset = transform_offset;
			const uint32_t translation_offset = rotation_offset + rotation_track_size;

			uint8_t* segment_data = m_data + segment.soa_start_offset;
			uint8_t* translation_data = segment_data + translation_offset;
			float* translations_x = safe_ptr_cast<float>(translation_data + offset_x);
			float* translations_y = safe_ptr_cast<float>(translation_data + offset_y);
			float* translations_z = safe_ptr_cast<float>(translation_data + offset_z);

			translations_x[sample_index] = vector_get_x(translation);
			translations_y[sample_index] = vector_get_y(translation);
			translations_z[sample_index] = vector_get_z(translation);
		}

		inline void ACL_SIMD_CALL track_database::set_scale(Vector4_32Arg0 scale, const segment_context& segment, uint32_t transform_index, uint32_t sample_index)
		{
			if (!m_has_scale)
				return;

			const uint32_t num_components_per_transform = get_num_components_per_transform(m_has_scale);
			const uint32_t num_simd_samples_per_track = segment.num_simd_samples_per_track;
			const uint32_t component_size = sizeof(float) * num_simd_samples_per_track;
			const uint32_t transform_size = component_size * num_components_per_transform;

			const uint32_t rotation_track_size = component_size * 4;
			const uint32_t translation_track_size = component_size * 3;

			const uint32_t offset_x = 0;
			const uint32_t offset_y = offset_x + component_size;
			const uint32_t offset_z = offset_y + component_size;

			const uint32_t transform_offset = transform_index * transform_size;

			const uint32_t rotation_offset = transform_offset;
			const uint32_t translation_offset = rotation_offset + rotation_track_size;
			const uint32_t scale_offset = translation_offset + translation_track_size;

			uint8_t* segment_data = m_data + segment.soa_start_offset;
			uint8_t* scale_data = segment_data + scale_offset;
			float* scales_x = safe_ptr_cast<float>(scale_data + offset_x);
			float* scales_y = safe_ptr_cast<float>(scale_data + offset_y);
			float* scales_z = safe_ptr_cast<float>(scale_data + offset_z);

			scales_x[sample_index] = vector_get_x(scale);
			scales_y[sample_index] = vector_get_y(scale);
			scales_z[sample_index] = vector_get_z(scale);
		}

#if 0
		inline uint32_t track_database::get_rotation_sample_num_bits(uint32_t transform_index) const
		{
			switch (m_rotation_format)
			{
			case RotationFormat8::Quat_128:				return sizeof(float) * 4 * 8;
			case RotationFormat8::QuatDropW_96:			return sizeof(float) * 3 * 8;
			case RotationFormat8::QuatDropW_48:			return sizeof(uint16_t) * 3 * 8;
			case RotationFormat8::QuatDropW_32:			return sizeof(uint32_t) * 8;
			case RotationFormat8::QuatDropW_Variable:	return get_num_bits_at_bit_rate(m_bit_rates[transform_index].rotation) * 3;
			default:
				ACL_ASSERT(false, "Invalid rotation format");
				return 0;
			}
		}

		inline uint32_t track_database::get_translation_sample_num_bits(uint32_t transform_index) const
		{
			switch (m_translation_format)
			{
			case VectorFormat8::Vector3_96:				return sizeof(float) * 3 * 8;
			case VectorFormat8::Vector3_48:				return sizeof(uint16_t) * 3 * 8;
			case VectorFormat8::Vector3_32:				return sizeof(uint32_t) * 8;
			case VectorFormat8::Vector3_Variable:		return get_num_bits_at_bit_rate(m_bit_rates[transform_index].translation) * 3;
			default:
				ACL_ASSERT(false, "Invalid translation format");
				return 0;
			}
		}

		inline uint32_t track_database::get_scale_sample_num_bits(uint32_t transform_index) const
		{
			switch (m_scale_format)
			{
			case VectorFormat8::Vector3_96:				return sizeof(float) * 3 * 8;
			case VectorFormat8::Vector3_48:				return sizeof(uint16_t) * 3 * 8;
			case VectorFormat8::Vector3_32:				return sizeof(uint32_t) * 8;
			case VectorFormat8::Vector3_Variable:		return get_num_bits_at_bit_rate(m_bit_rates[transform_index].scale) * 3;
			default:
				ACL_ASSERT(false, "Invalid scale format");
				return 0;
			}
		}
#endif
	}
}

ACL_IMPL_FILE_PRAGMA_POP
