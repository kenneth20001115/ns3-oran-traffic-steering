#include "my_lm.h"
#include "oran-command-lte-2-lte-handover.h"
#include "oran-data-repository.h"
#include <ns3/abort.h>
#include <ns3/log.h>
#include <ns3/simulator.h>
#include <ns3/uinteger.h>

#include<iostream>
#include <cfloat>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("MyLm");

NS_OBJECT_ENSURE_REGISTERED(MyLm);

TypeId
MyLm::GetTypeId(void)
{
    static TypeId tid = TypeId("ns3::MyLm")
                            .SetParent<OranLm>()
                            .AddConstructor<MyLm>();

    return tid;
}

MyLm::MyLm(void)
    : OranLm()
{   
    NS_LOG_FUNCTION(this);
    runTime=0;
    vgd.setVar(5.0,0.01,0.1);
    M=5.0;
    throughputRun=0;
    throughputTotal=0;
    m_name = "MyLm";
}

MyLm::~MyLm(void)
{
    NS_LOG_FUNCTION(this);
}

std::vector<Ptr<OranCommand>>
MyLm::Run(void)
{
    NS_LOG_FUNCTION(this);
    runTime++;
    
    std::vector<Ptr<OranCommand>> commands;

    if (m_active)
    {
        NS_ABORT_MSG_IF(m_nearRtRic == nullptr,
                        "Attempting to run LM (" + m_name + ") with NULL Near-RT RIC");
        if(runTime%6==1){
            vgd.updateMb();
            M=vgd.getMb();
            std::cout<<"M: "<<M<<std::endl;;
        }
        Ptr<OranDataRepository> data = m_nearRtRic->Data();
        std::vector<UeInfo> ueInfos = GetUeInfos(data);
        std::vector<EnbInfo> enbInfos = GetEnbInfos(data);
        commands = GetHandoverCommands(data, ueInfos, enbInfos);
        throughputRun+=GetThroughput(ueInfos);
        throughputTotal+=GetThroughput(ueInfos);
        totalloss += GetLoss(ueInfos);
        if(runTime%6==0){
            vgd.updateM(throughputRun);
            throughputRun=0;
        }
        std::cout<<"total: "<<(throughputTotal*1500*8/1000)/(runTime*5+1)<<std::endl;
        std::cout<<"total loss: "<<totalloss/runTime<<std::endl;
    }
    
    // Return the commands.
    return commands;
}

std::vector<MyLm::UeInfo>
MyLm::GetUeInfos(Ptr<OranDataRepository> data) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<UeInfo> ueInfos;
    for (auto ueId : data->GetLteUeE2NodeIds())
    {
        UeInfo ueInfo;
        ueInfo.nodeId = ueId;
        // Get the current cell ID and RNTI of the UE and record it.
        bool found;
        std::tie(found, ueInfo.cellId, ueInfo.rnti) = data->GetLteUeCellInfo(ueInfo.nodeId);
        if (found)
        {
            // Get the latest location of the UE.
            std::map<Time, Vector> nodePositions =
                data->GetNodePositions(ueInfo.nodeId, Seconds(0), Simulator::Now());

            if (!nodePositions.empty())
            {
                // We found both the cell and location informtaion for this UE
                // so record it for a later analysis.
                ueInfo.position = nodePositions.rbegin()->second;
                ueInfo.loss = data->GetAppLoss(ueInfo.nodeId);
                ueInfo.Rx = data->GetRx(ueInfo.nodeId);
                ueInfo.Tx = data->GetTx(ueInfo.nodeId);
                std::cout<<"ue "<<ueInfo.nodeId<<" loss :"<<ueInfo.loss<<" Tx: "<<ueInfo.Tx<<" Rx: "<<ueInfo.Rx<<std::endl;
                ueInfos.push_back(ueInfo);
            }
            else
            {
                NS_LOG_INFO("Could not find LTE UE location for E2 Node ID = " << ueInfo.nodeId);
            }
        }
        else
        {
            NS_LOG_INFO("Could not find LTE UE cell info for E2 Node ID = " << ueInfo.nodeId);
        }
    }
    return ueInfos;
}

std::vector<MyLm::EnbInfo>
MyLm::GetEnbInfos(Ptr<OranDataRepository> data) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<EnbInfo> enbInfos;
    for (auto enbId : data->GetLteEnbE2NodeIds())
    {
        EnbInfo enbInfo;
        enbInfo.nodeId = enbId;
        // Get the cell ID of this eNB and record it.
        bool found;
        std::tie(found, enbInfo.cellId) = data->GetLteEnbCellInfo(enbInfo.nodeId);
        if (found)
        {
            // Get all known locations of the eNB.
            std::map<Time, Vector> nodePositions =
                data->GetNodePositions(enbInfo.nodeId, Seconds(0), Simulator::Now());

            if (!nodePositions.empty())
            {
                // We found both the cell and location information for this
                // eNB so record it for a later analysis.
                enbInfo.position = nodePositions.rbegin()->second;
                enbInfos.push_back(enbInfo);
            }
            else
            {
                NS_LOG_INFO("Could not find LTE eNB location for E2 Node ID = " << enbInfo.nodeId);
            }
        }
        else
        {
            NS_LOG_INFO("Could not find LTE eNB cell info for E2 Node ID = " << enbInfo.nodeId);
        }
    }
    return enbInfos;
}

std::vector<Ptr<OranCommand>>
MyLm::GetHandoverCommands(
    Ptr<OranDataRepository> data,
    std::vector<MyLm::UeInfo> ueInfos,
    std::vector<MyLm::EnbInfo> enbInfos) const
{
    NS_LOG_FUNCTION(this << data);

    std::vector<Ptr<OranCommand>> commands;
    double myLoss=0;

    for (auto ueInfo : ueInfos)
    {
        double max = -DBL_MAX;               
        uint64_t oldCellNodeId;             // The ID of the cell currently serving the UE.
        uint16_t newCellId = ueInfo.cellId; // The ID of the closest cell.
        auto rsrpMeasurements = data->GetLteUeRsrpRsrq(ueInfo.nodeId);
        for (auto rsrpMeasurement : rsrpMeasurements)
        {
            uint16_t rnti;
            uint16_t cellId;
            double rsrp;
            double rsrq;
            bool isServingCell;
            uint16_t componentCarrierId;
            std::tie(rnti, cellId, rsrp, rsrq, isServingCell, componentCarrierId) = rsrpMeasurement;
            LogLogicToRepository("RSRP from UE with RNTI " + std::to_string(rnti) +
                                 " in CellID " + std::to_string(ueInfo.cellId) +
                                 " to eNB with CellID " + std::to_string(cellId) + " is " +
                                 std::to_string(rsrp));
            //std::cout<<"sinr from UE with RNTI " << std::to_string(rnti) <<
            //                     " in CellID " << std::to_string(ueInfo.cellId) <<
            //                     " to eNB with CellID " << std::to_string(cellId) << " is " +
            //                     std::to_string(rsrp)<<std::endl;

            if (rsrp > max + M) 
            {
                // Record the new maximum
                max = rsrp;
                // Record the ID of the cell that produced the new maximum.
                newCellId = cellId;

                LogLogicToRepository("RSRP to eNB with CellID " +
                                     std::to_string(cellId) + " is largest so far");
            }
        }

        for (const auto& enbInfo : enbInfos)
        {
            // Check if this cell is the currently serving this UE.
            if (ueInfo.cellId == enbInfo.cellId)
            {
                // It is, so indicate record the ID of the cell that is
                // currently serving the UE.
                oldCellNodeId = enbInfo.nodeId;
            }
        }

        // Check if the ID of the closest cell is different from ID of the cell
        // that is currently serving the UE
        if (newCellId != ueInfo.cellId)
        {
            // It is, so issue a handover command.
            Ptr<OranCommandLte2LteHandover> handoverCommand =
                CreateObject<OranCommandLte2LteHandover>();
            // Send the command to the cell currently serving the UE.
            handoverCommand->SetAttribute("TargetE2NodeId", UintegerValue(oldCellNodeId));
            // Use the RNTI that the current cell is using to identify the UE.
            handoverCommand->SetAttribute("TargetRnti", UintegerValue(ueInfo.rnti));
            // Give the current cell the ID of the new cell to handover to.
            handoverCommand->SetAttribute("TargetCellId", UintegerValue(newCellId));
            // Log the command to the storage
            data->LogCommandLm(m_name, handoverCommand);
            // Add the command to send.
            commands.push_back(handoverCommand);

            LogLogicToRepository("eNB (CellID " + std::to_string(newCellId) + ")" +
                                 " is different than the currently attached eNB" + " (CellID " +
                                 std::to_string(ueInfo.cellId) + ")." +
                                 " Issuing handover command.");
        }
        myLoss+=ueInfo.loss;
        rx+=ueInfo.Rx;
    }
    //std::cout<<"total loss: "<<myLoss/4.0<<std::endl;
    std::cout<<"total Rx: "<<rx<<std::endl;
    return commands;
}
double MyLm::GetThroughput(std::vector<MyLm::UeInfo> ueInfos) const{
        int totalRx=0;
        for (auto ueInfo : ueInfos)
            totalRx+=ueInfo.Rx;
        return totalRx;
}

double MyLm::GetLoss(std::vector<MyLm::UeInfo> ueInfos) const{
         double trx=0;
         double ttx=0;
        for (auto ueInfo : ueInfos){
            ttx+=ueInfo.Tx;
            trx+=ueInfo.Rx;
            }
        return (ttx-trx)/ttx;
}

} // namespace ns3

int main(){
    return 0;
}
