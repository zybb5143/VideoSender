#include <string>
#include <algorithm>
std::string websocketURLCreator(std::string hostIP, int port, std::string path, bool isWss, std::initializer_list<std::pair<std::string, std::string>> list)
{
    std::string webSocketURL = (isWss ? "wss://" : "ws://") + hostIP + ":" + std::to_string(port) + "/" + path;
    if (list.size() > 0)
    {
        webSocketURL += "?";
        for_each(list.begin(), list.end(), [&](const auto &p)
                 { webSocketURL += p.first + "=" + p.second + "&"; });
        webSocketURL.pop_back();
    }
    return webSocketURL;
}