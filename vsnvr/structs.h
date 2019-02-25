#include <utility>

#include <utility>

#pragma once

#include <string>
using std::string;

struct LinkInfo {
    LinkInfo(string a, string i, int p, int c, string u, string w): alias(std::move(a)),
    ip(std::move(i)), port(p), channel(c), username(std::move(u)), password(std::move(w)) {}
    string alias;
    string ip;
    int port;
    int channel;
    string username;
    string password;
};

struct RealPlayHandle {
    RealPlayHandle(int u = -1, int r = -1) : userId(u), realHandle(r) {}
    int userId = -1;
    int realHandle = -1;
};