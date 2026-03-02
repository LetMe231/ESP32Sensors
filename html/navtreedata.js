/*
 @licstart  The following is the entire license notice for the JavaScript code in this file.

 The MIT License (MIT)

 Copyright (C) 1997-2020 by Dimitri van Heesch

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute,
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or
 substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 @licend  The above is the entire license notice for the JavaScript code in this file
*/
var NAVTREE =
[
  [ "BLE_ESP_Sens", "index.html", [
    [ "Multi-Variant ESP32-S3 BLE Mesh Firmware", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html", [
      [ "Quick Start", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md1", [
        [ "Build for Standard Configuration (Both Sensors)", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md2", null ],
        [ "Build for AHT20 Only (Temp/Humidity)", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md3", null ],
        [ "Build for MAX30101 Only (Heart Rate/SpO2)", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md4", null ],
        [ "Build with Custom Pins", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md5", null ]
      ] ],
      [ "Configuration System", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md6", [
        [ "How It Works", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md7", null ],
        [ "Configuration Hierarchy", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md8", null ]
      ] ],
      [ "Available Build Environments", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md9", [
        [ "1. <b>esp32s3</b> (DEFAULT - Both Sensors)", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md10", null ],
        [ "2. <b>esp32s3-aht20-only</b> (AHT20 ONLY)", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md11", null ],
        [ "3. <b>esp32s3-max30101-only</b> (MAX30101 ONLY)", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md12", null ],
        [ "4. <b>esp32s3-custom</b> (TEMPLATE - Edit for Your Pins)", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md13", null ]
      ] ],
      [ "Configuration Reference", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md14", [
        [ "config.h Flags", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md15", null ],
        [ "How to Override Defaults", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md16", null ]
      ] ],
      [ "Payload Format", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md17", null ],
      [ "Compile-Time vs Runtime Configuration", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md18", null ],
      [ "Troubleshooting", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md19", [
        [ "Build fails with \"undefined reference to axt20_init\"", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md20", null ],
        [ "I2C address conflicts (both sensors on same bus)", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md21", null ],
        [ "GPIO pin conflicts", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md22", null ],
        [ "Sensor stuck during initialization", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md23", null ]
      ] ],
      [ "Advanced Usage", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md24", [
        [ "Adding a Third Sensor", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md25", null ],
        [ "Creating an \"All Disabled\" Build", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md26", null ]
      ] ],
      [ "Building for Multiple Devices", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md27", [
        [ "Workflow", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md28", null ],
        [ "Using pio project config", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md29", null ]
      ] ],
      [ "Summary", "md__b_u_i_l_d___v_a_r_i_a_n_t_s.html#autotoc_md30", null ]
    ] ],
    [ "README", "md__r_e_a_d_m_e.html", [
      [ "ESP BLE Mesh Vendor Server Example", "md__r_e_a_d_m_e.html#autotoc_md31", null ]
    ] ]
  ] ]
];

var NAVTREEINDEX =
[
"index.html"
];

var SYNCONMSG = 'click to disable panel synchronization';
var SYNCOFFMSG = 'click to enable panel synchronization';
var LISTOFALLMEMBERS = 'List of all members';