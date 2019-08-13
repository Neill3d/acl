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
#include "acl/core/track_traits.h"
#include "acl/core/track_types.h"

#include <cstdint>

ACL_IMPL_FILE_PRAGMA_PUSH

namespace acl
{
	class track
	{
	public:
		track()
			: m_allocator(nullptr)
			, m_data(nullptr)
			, m_num_samples(0)
			, m_stride(0)
			, m_data_size(0)
			, m_sample_rate(0.0f)
			, m_type(track_type8::float1f)
			, m_category(track_category8::scalarf)
			, m_element_size(0)
			, m_desc()
		{}

		track(track&& other)
			: m_allocator(other.m_allocator)
			, m_data(other.m_data)
			, m_num_samples(other.m_num_samples)
			, m_stride(other.m_stride)
			, m_data_size(other.m_data_size)
			, m_sample_rate(other.m_sample_rate)
			, m_type(other.m_type)
			, m_category(other.m_category)
			, m_element_size(other.m_element_size)
			, m_desc(other.m_desc)
		{
			other.m_allocator = nullptr;
			other.m_data = nullptr;
		}

		~track()
		{
			if (is_owner())
			{
				// We own the memory, free it
				m_allocator->deallocate(m_data, m_data_size);
			}
		}

		track& operator=(track&& other)
		{
			std::swap(m_allocator, other.m_allocator);
			std::swap(m_data, other.m_data);
			std::swap(m_num_samples, other.m_num_samples);
			std::swap(m_stride, other.m_stride);
			std::swap(m_data_size, other.m_data_size);
			std::swap(m_sample_rate, other.m_sample_rate);
			std::swap(m_type, other.m_type);
			std::swap(m_category, other.m_category);
			std::swap(m_element_size, other.m_element_size);
			std::swap(m_desc, other.m_desc);
			return *this;
		}

		void* operator[](uint32_t index)
		{
			// If we have an allocator, we own the memory and mutable pointers are allowed
			ACL_ASSERT(is_owner(), "Mutable reference not allowed, create a copy instead");
			ACL_ASSERT(index < m_num_samples, "Invalid sample index. %u >= %u", index, m_num_samples);
			return m_allocator ? m_data + (index * m_stride) : nullptr;
		}

		const void* operator[](uint32_t index) const
		{
			ACL_ASSERT(index < m_num_samples, "Invalid sample index. %u >= %u", index, m_num_samples);
			return m_data + (index * m_stride);
		}

		bool is_owner() const { return m_allocator != nullptr; }
		bool is_ref() const { return m_allocator == nullptr; }

		uint32_t get_num_samples() const { return m_num_samples; }
		uint32_t get_stride() const { return m_stride; }
		track_type8 get_type() const { return m_type; }
		track_category8 get_category() const { return m_category; }
		uint32_t get_element_size() const { return m_element_size; }
		float get_sample_rate() const { return m_sample_rate; }
		uint32_t get_output_index() const
		{
			switch (m_category)
			{
			default:
			case track_category8::scalarf:	return m_desc.scalar.output_index;
			}
		}

		template<typename desc_type>
		desc_type& get_description()
		{
			ACL_ASSERT(desc_type::category == m_category, "Unexpected track category");
			switch (desc_type::category)
			{
			default:
			case track_category8::scalarf:	return m_desc.scalar;
			}
		}

		template<typename desc_type>
		const desc_type& get_description() const
		{
			ACL_ASSERT(desc_type::category == m_category, "Unexpected track category");
			switch (desc_type::category)
			{
			default:
			case track_category8::scalarf:	return m_desc.scalar;
			}
		}

		track get_copy(IAllocator& allocator) const
		{
			track track_;
			track_.m_allocator = &allocator;
			track_.m_data = reinterpret_cast<uint8_t*>(allocator.allocate(m_data_size));
			track_.m_num_samples = m_num_samples;
			track_.m_stride = m_stride;
			track_.m_data_size = m_data_size;
			track_.m_type = m_type;
			track_.m_category = m_category;
			track_.m_desc = m_desc;

			std::memcpy(track_.m_data, m_data, m_data_size);

			return track_;
		}

		track get_ref() const
		{
			track track_;
			track_.m_allocator = nullptr;
			track_.m_data = m_data;
			track_.m_num_samples = m_num_samples;
			track_.m_stride = m_stride;
			track_.m_data_size = m_data_size;
			track_.m_type = m_type;
			track_.m_category = m_category;
			track_.m_desc = m_desc;
			return track_;
		}

	protected:
		track(const track&) = delete;
		track& operator=(const track&) = delete;

		track(IAllocator* allocator, uint8_t* data, uint32_t num_samples, uint32_t stride, size_t data_size, float sample_rate, track_type8 type, track_category8 category, uint8_t element_size)
			: m_allocator(allocator)
			, m_data(data)
			, m_num_samples(num_samples)
			, m_stride(stride)
			, m_data_size(data_size)
			, m_sample_rate(sample_rate)
			, m_type(type)
			, m_category(category)
			, m_element_size(element_size)
			, m_desc()
		{}

		IAllocator*				m_allocator;
		uint8_t*				m_data;

		uint32_t				m_num_samples;
		uint32_t				m_stride;
		size_t					m_data_size;

		float					m_sample_rate;

		track_type8				m_type;
		track_category8			m_category;
		uint16_t				m_element_size;

		union desc_union
		{
			track_desc_scalarf	scalar;
			// TODO: Add other description types here

			desc_union() : scalar() {}
			desc_union(const track_desc_scalarf& desc) : scalar(desc) {}
		};

		desc_union				m_desc;
	};

	template<track_type8 track_type_>
	class track_typed final : public track
	{
	public:
		static constexpr track_type8 type = track_type_;
		static constexpr track_category8 category = track_traits<track_type_>::category;

		using element_type = typename track_traits<track_type_>::type;
		using desc_type = typename track_traits<track_type_>::desc_type;

		track_typed()
			: track()
		{
			static_assert(sizeof(track_typed) == sizeof(track), "You cannot add member variables to this class");
		}

		track_typed(track_typed&& other) : track(std::forward<track>(other)) {}

		track_typed& operator=(track_typed&& other) { return track::operator=(std::forward<track>(other)); }

		element_type& operator[](uint32_t index)
		{
			// If we have an allocator, we own the memory and mutable references are allowed
			ACL_ASSERT(is_owner(), "Mutable reference not allowed, create a copy instead");
			ACL_ASSERT(index < m_num_samples, "Invalid sample index. %u >= %u", index, m_num_samples);
			return m_allocator ? *reinterpret_cast<element_type*>(m_data + (index * m_stride)) : *reinterpret_cast<element_type*>(0x42);
		}

		const element_type& operator[](uint32_t index) const
		{
			ACL_ASSERT(index < m_num_samples, "Invalid sample index. %u >= %u", index, m_num_samples);
			return *reinterpret_cast<const element_type*>(m_data + (index * m_stride));
		}

		desc_type& get_description()
		{
			switch (category)
			{
			default:
			case track_category8::scalarf:	return m_desc.scalar;
			}
		}

		const desc_type& get_description() const
		{
			switch (category)
			{
			default:
			case track_category8::scalarf:	return m_desc.scalar;
			}
		}

		track_category8 get_category() const { return category; }

		// Copies the data and owns the copy
		static track_typed<track_type_> make_copy(const desc_type& desc, IAllocator& allocator, const element_type* data, uint32_t num_samples, float sample_rate, uint32_t stride = sizeof(element_type))
		{
			const size_t data_size = size_t(num_samples) * sizeof(element_type);
			const uint8_t* data_raw = reinterpret_cast<const uint8_t*>(data);

			// Copy the data manually to avoid preserving the stride
			element_type* data_copy = reinterpret_cast<element_type*>(allocator.allocate(data_size));
			for (uint32_t index = 0; index < num_samples; ++index)
				data_copy[index] = *reinterpret_cast<element_type*>(data_raw + (index * stride));

			return track_typed<track_type_>(&allocator, reinterpret_cast<uint8_t*>(data_copy), num_samples, sizeof(element_type), data_size, sample_rate, desc);
		}

		// Preallocates but does not initialize the data and owns it
		static track_typed<track_type_> make_reserve(const desc_type& desc, IAllocator& allocator, uint32_t num_samples, float sample_rate)
		{
			const size_t data_size = size_t(num_samples) * sizeof(element_type);
			return track_typed<track_type_>(&allocator, reinterpret_cast<uint8_t*>(allocator.allocate(data_size)), num_samples, sizeof(element_type), data_size, sample_rate, desc);
		}

		// Takes ownership of the already allocated data
		static track_typed<track_type_> make_owner(const desc_type& desc, IAllocator& allocator, element_type* data, uint32_t num_samples, float sample_rate, uint32_t stride = sizeof(element_type))
		{
			const size_t data_size = size_t(num_samples) * stride;
			return track_typed<track_type_>(&allocator, reinterpret_cast<uint8_t*>(data), num_samples, stride, data_size, sample_rate, desc);
		}

		// Just references the data without owning it
		static track_typed<track_type_> make_ref(const desc_type& desc, const element_type* data, uint32_t num_samples, float sample_rate, uint32_t stride = sizeof(element_type))
		{
			const size_t data_size = size_t(num_samples) * stride;
			return track_typed<track_type_>(nullptr, const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(data)), num_samples, stride, data_size, sample_rate, desc);
		}

	private:
		track_typed(const track_typed&) = delete;
		track_typed& operator=(const track_typed&) = delete;

		track_typed(IAllocator* allocator, uint8_t* data, uint32_t num_samples, uint32_t stride, size_t data_size, float sample_rate, const desc_type& desc)
			: track(allocator, data, num_samples, stride, data_size, sample_rate, type, category, sizeof(element_type))
		{
			m_desc = desc;
		}
	};

	template<typename track_type>
	inline track_type& track_cast(track& track_)
	{
		ACL_ASSERT(track_type::type == track_.get_type(), "Unexpected track type");
		return static_cast<track_type&>(track_);
	}

	template<typename track_type>
	inline const track_type& track_cast(const track& track_)
	{
		ACL_ASSERT(track_type::type == track_.get_type(), "Unexpected track type");
		return static_cast<const track_type&>(track_);
	}

	template<typename track_type>
	inline track_type* track_cast(track* track_)
	{
		if (track_ == nullptr || track_type::type != track_->get_type())
			return nullptr;

		return static_cast<track_type*>(track_);
	}

	template<typename track_type>
	inline const track_type* track_cast(const track* track_)
	{
		if (track_ == nullptr || track_type::type != track_->get_type())
			return nullptr;

		return static_cast<const track_type*>(track_);
	}

	//////////////////////////////////////////////////////////////////////////

	using track_float1f			= track_typed<track_type8::float1f>;
	using track_float2f			= track_typed<track_type8::float2f>;
	using track_float3f			= track_typed<track_type8::float3f>;
	using track_float4f			= track_typed<track_type8::float4f>;
	using track_vector4f		= track_typed<track_type8::vector4f>;
}

ACL_IMPL_FILE_PRAGMA_POP
