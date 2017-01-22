#include <networktables/NetworkTable.h>
#include <unistd.h>
#include <memory>

int main() {
	std::shared_ptr<NetworkTable> table;

	table = NetworkTable::GetTable("vision");

	while(1) {
		table->PutNumber("X", 1);
		sleep(1);
	}

	return 0;
}
