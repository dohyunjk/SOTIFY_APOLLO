/******************************************************************************
 * Copyright 2022 The Syssec Reseacher. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "absl/strings/match.h"
#include "gflags/gflags.h"

#include "modules/map/proto/map.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/map/hdmap/adapter/opendrive_adapter.h"
#include "modules/map/hdmap/hdmap_util.h"

/**
 * A map tool to generate a map with adjusted speed limit
 */

DEFINE_string(output_dir, "/tmp/", "output map directory");
DEFINE_double(speed_limit, 10.0, "speed limit to be set");

using apollo::cyber::common::GetProtoFromFile;
using apollo::hdmap::Map;
using apollo::hdmap::adapter::OpendriveAdapter;

static void AdjustSpeedLimit(Map* map_pb) {
	for (int i = 0; i < map_pb->lane_size(); ++i) {
		auto* lane = map_pb->mutable_lane(i);

		lane->set_speed_limit(FLAGS_speed_limit);

		AINFO << "Adjust speed limit of lane " << lane->id().id()
			<< " as " << FLAGS_speed_limit << "m/s";
	}
}

static void OutputMap(const Map& map_pb) {
	std::ofstream map_txt_file(FLAGS_output_dir + "/base_map.txt");
	map_txt_file << map_pb.DebugString();
	map_txt_file.close();

	std::ofstream map_bin_file(FLAGS_output_dir + "/base_map.bin");
	std::string map_str;
	map_pb.SerializeToString(&map_str);
	map_bin_file << map_str;
	map_bin_file.close();
}

int main(int32_t argc, char** argv) {
	google::InitGoogleLogging(argv[0]);
	FLAGS_alsologtostderr = true;
	FLAGS_v = 3;

	google::ParseCommandLineFlags(&argc, &argv, true);

	Map map_pb;
	const auto map_file = apollo::hdmap::BaseMapFile();
	if (absl::EndsWith(map_file, ".xml")) {
		ACHECK(OpendriveAdapter::LoadData(map_file, &map_pb));
	} else {
		ACHECK(GetProtoFromFile(map_file, &map_pb)) << "Fail to open: " << map_file;
	}

	AdjustSpeedLimit(&map_pb);
	OutputMap(map_pb);
	AINFO << "adjusted_bin_map generated at:" << FLAGS_output_dir;

	return 0;
}
