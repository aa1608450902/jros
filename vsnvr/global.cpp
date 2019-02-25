#include "global.h"

QList<RealPlayHandle>& gRealPlayHandle()
{static QList<RealPlayHandle> rp; return rp;}

QList<LinkInfo>& gLinkInfo()
{static QList<LinkInfo> l; return l;}