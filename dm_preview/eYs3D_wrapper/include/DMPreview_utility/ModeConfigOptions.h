#ifndef MODECONFIGOPTIONS_H
#define MODECONFIGOPTIONS_H
#include <cstdio>
#include <algorithm>
#include <ModeConfig.h>
#ifdef WIN32
#  include <eSPDI_Common.h>
#else
#  include "eSPDI_def.h"
#endif

class ModeConfigOptions
{
public:
    ModeConfigOptions(USB_PORT_TYPE usbType, unsigned short nPID);
    ~ModeConfigOptions() = default;

    int GetModeCount(){ return m_modeConfigs.size(); }
    std::vector<ModeConfig::MODE_CONFIG> GetModes(){ return m_modeConfigs; }

    int SelectCurrentIndex(size_t nIndex){
        size_t nVecIndex = TransformDBtoVec(nIndex);
        if(nVecIndex >= m_modeConfigs.size() ) return APC_NullPtr;
        m_nCurrentIndex = nVecIndex;
        return APC_OK;
    }
    int GetCurrentIndex(){ return TransformVectoDB(m_nCurrentIndex); }
    ModeConfig::MODE_CONFIG GetCurrentModeInfo()
    {
        ModeConfig::MODE_CONFIG empty;
        if(EOF == m_nCurrentIndex) return empty;
        return m_modeConfigs[m_nCurrentIndex];
    }

private:
    std::vector< ModeConfig::MODE_CONFIG > m_modeConfigs;
    std::map <int, int> m_DBVecMap;
    int m_nCurrentIndex;
    int m_nVecIndex = 0;
    int TransformDBtoVec(int nDBIndex) {
        auto iter = m_DBVecMap.find(nDBIndex);
        if (iter != m_DBVecMap.end()) return iter->second;
        else return m_modeConfigs.size();
    }
    int TransformVectoDB(int nVecIndex) {
        auto findKey = std::find_if(std::begin(m_DBVecMap), std::end(m_DBVecMap), [&](const std::pair<int, int>&pair){ return pair.second == nVecIndex;});
        if( findKey != std::end(m_DBVecMap)) return findKey->first;
    }
};

#endif // MODECONFIGOPTIONS_H
