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
#include "acl/core/track_types.h"

#include <rtm/types.h>

#include <cstdint>

ACL_IMPL_FILE_PRAGMA_PUSH

namespace acl
{
	template<track_type8 track_type>
	struct track_traits {};

	template<>
	struct track_traits<track_type8::float1f>
	{
		using type = float;
		static constexpr track_category8 category = track_category8::scalarf;
		using desc_type = track_desc_scalarf;
	};

	template<>
	struct track_traits<track_type8::float2f>
	{
		using type = rtm::float2f;
		static constexpr track_category8 category = track_category8::scalarf;
		using desc_type = track_desc_scalarf;
	};

	template<>
	struct track_traits<track_type8::float3f>
	{
		using type = rtm::float3f;
		static constexpr track_category8 category = track_category8::scalarf;
		using desc_type = track_desc_scalarf;
	};

	template<>
	struct track_traits<track_type8::float4f>
	{
		using type = rtm::float4f;
		static constexpr track_category8 category = track_category8::scalarf;
		using desc_type = track_desc_scalarf;
	};

	template<>
	struct track_traits<track_type8::vector4f>
	{
		using type = rtm::vector4f;
		static constexpr track_category8 category = track_category8::scalarf;
		using desc_type = track_desc_scalarf;
	};
}

ACL_IMPL_FILE_PRAGMA_POP
