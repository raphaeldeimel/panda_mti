#include <string>

class netft
{
public:
    netft();
    int connect_netbox(std::string address);
    int start_netbox();
    int getRDT();
};
