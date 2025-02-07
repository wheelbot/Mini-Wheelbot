/*
 * Copyright (c) 2021, Niklas Hauser
 * Copyright (c) 2022, Andrey Kunitsyn
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include "data.hpp"
#include "static.hpp"

namespace modm::platform
{

/// @ingroup modm_platform_gpio
/// @{
using GpioA0 = GpioStatic<detail::DataA0>;
using GpioOutputA0 = GpioA0;
using GpioInputA0  = GpioA0;

using GpioA1 = GpioStatic<detail::DataA1>;
using GpioOutputA1 = GpioA1;
using GpioInputA1  = GpioA1;

using GpioA2 = GpioStatic<detail::DataA2>;
using GpioOutputA2 = GpioA2;
using GpioInputA2  = GpioA2;

using GpioA3 = GpioStatic<detail::DataA3>;
using GpioOutputA3 = GpioA3;
using GpioInputA3  = GpioA3;

using GpioA4 = GpioStatic<detail::DataA4>;
using GpioOutputA4 = GpioA4;
using GpioInputA4  = GpioA4;

using GpioA5 = GpioStatic<detail::DataA5>;
using GpioOutputA5 = GpioA5;
using GpioInputA5  = GpioA5;

using GpioA6 = GpioStatic<detail::DataA6>;
using GpioOutputA6 = GpioA6;
using GpioInputA6  = GpioA6;

using GpioA7 = GpioStatic<detail::DataA7>;
using GpioOutputA7 = GpioA7;
using GpioInputA7  = GpioA7;

using GpioA8 = GpioStatic<detail::DataA8>;
using GpioOutputA8 = GpioA8;
using GpioInputA8  = GpioA8;

using GpioA9 = GpioStatic<detail::DataA9>;
using GpioOutputA9 = GpioA9;
using GpioInputA9  = GpioA9;

using GpioA10 = GpioStatic<detail::DataA10>;
using GpioOutputA10 = GpioA10;
using GpioInputA10  = GpioA10;

using GpioA11 = GpioStatic<detail::DataA11>;
using GpioOutputA11 = GpioA11;
using GpioInputA11  = GpioA11;

using GpioA12 = GpioStatic<detail::DataA12>;
using GpioOutputA12 = GpioA12;
using GpioInputA12  = GpioA12;

using GpioA13 = GpioStatic<detail::DataA13>;
using GpioOutputA13 = GpioA13;
using GpioInputA13  = GpioA13;

using GpioA14 = GpioStatic<detail::DataA14>;
using GpioOutputA14 = GpioA14;
using GpioInputA14  = GpioA14;

using GpioA15 = GpioStatic<detail::DataA15>;
using GpioOutputA15 = GpioA15;
using GpioInputA15  = GpioA15;

using GpioB0 = GpioStatic<detail::DataB0>;
using GpioOutputB0 = GpioB0;
using GpioInputB0  = GpioB0;

using GpioB1 = GpioStatic<detail::DataB1>;
using GpioOutputB1 = GpioB1;
using GpioInputB1  = GpioB1;

using GpioB2 = GpioStatic<detail::DataB2>;
using GpioOutputB2 = GpioB2;
using GpioInputB2  = GpioB2;

using GpioB3 = GpioStatic<detail::DataB3>;
using GpioOutputB3 = GpioB3;
using GpioInputB3  = GpioB3;

using GpioB4 = GpioStatic<detail::DataB4>;
using GpioOutputB4 = GpioB4;
using GpioInputB4  = GpioB4;

using GpioB5 = GpioStatic<detail::DataB5>;
using GpioOutputB5 = GpioB5;
using GpioInputB5  = GpioB5;

using GpioB6 = GpioStatic<detail::DataB6>;
using GpioOutputB6 = GpioB6;
using GpioInputB6  = GpioB6;

using GpioB7 = GpioStatic<detail::DataB7>;
using GpioOutputB7 = GpioB7;
using GpioInputB7  = GpioB7;

using GpioB8 = GpioStatic<detail::DataB8>;
using GpioOutputB8 = GpioB8;
using GpioInputB8  = GpioB8;

using GpioB9 = GpioStatic<detail::DataB9>;
using GpioOutputB9 = GpioB9;
using GpioInputB9  = GpioB9;

using GpioB10 = GpioStatic<detail::DataB10>;
using GpioOutputB10 = GpioB10;
using GpioInputB10  = GpioB10;

using GpioB11 = GpioStatic<detail::DataB11>;
using GpioOutputB11 = GpioB11;
using GpioInputB11  = GpioB11;

using GpioB12 = GpioStatic<detail::DataB12>;
using GpioOutputB12 = GpioB12;
using GpioInputB12  = GpioB12;

using GpioB13 = GpioStatic<detail::DataB13>;
using GpioOutputB13 = GpioB13;
using GpioInputB13  = GpioB13;

using GpioB14 = GpioStatic<detail::DataB14>;
using GpioOutputB14 = GpioB14;
using GpioInputB14  = GpioB14;

using GpioB15 = GpioStatic<detail::DataB15>;
using GpioOutputB15 = GpioB15;
using GpioInputB15  = GpioB15;

using GpioC13 = GpioStatic<detail::DataC13>;
using GpioOutputC13 = GpioC13;
using GpioInputC13  = GpioC13;

using GpioC14 = GpioStatic<detail::DataC14>;
using GpioOutputC14 = GpioC14;
using GpioInputC14  = GpioC14;

using GpioC15 = GpioStatic<detail::DataC15>;
using GpioOutputC15 = GpioC15;
using GpioInputC15  = GpioC15;

using GpioF0 = GpioStatic<detail::DataF0>;
using GpioOutputF0 = GpioF0;
using GpioInputF0  = GpioF0;

using GpioF1 = GpioStatic<detail::DataF1>;
using GpioOutputF1 = GpioF1;
using GpioInputF1  = GpioF1;

using GpioG10 = GpioStatic<detail::DataG10>;
using GpioOutputG10 = GpioG10;
using GpioInputG10  = GpioG10;

/// @}

} // namespace modm::platform

