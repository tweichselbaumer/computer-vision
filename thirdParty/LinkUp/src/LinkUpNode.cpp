#include "LinkUpNode.h"

uint16_t LinkUpNode::getRaw(uint8_t* pData, uint16_t nMax)
{
	uint16_t nResult;
	nResult = connector.getRaw(pData, nMax);
	return nResult;
}

void LinkUpNode::lock()
{
#ifdef LINKUP_BOOST_THREADSAFE
	mtx.lock();
#endif
}

void LinkUpNode::unlock()
{
#ifdef LINKUP_BOOST_THREADSAFE
	mtx.unlock();
#endif
}

void LinkUpNode::reset() {
	timestamps.nPingTimeout = 0;
	timestamps.nInitTryTimeout = 0;
	progress(NULL, 0, LinkUpProgressType::Normal);
	progress(NULL, 0, LinkUpProgressType::Normal);
}

void LinkUpNode::progress(uint8_t* pData, uint16_t nCount, LinkUpProgressType nProgressType)
{

	if (nProgressType & LinkUpProgressType::Input)
	{
#ifdef LINKUP_DEBUG_DETAIL
		if (nCount > 0)
			std::cout << nCount << std::endl;
#endif //LINKUP_DEBUG_DETAIL
		connector.progress(pData, nCount);
	}

	if (nProgressType & LinkUpProgressType::Normal)
	{


		uint32_t nTime = getSystemTime();

		int i = 0;

		while (connector.hasNext()) 
		{
			LinkUpPacket packet = connector.next();
			receivedPacket(packet, nTime);
			i++;
		}

#ifdef LINKUP_DEBUG
		/*if (i > 0)
			cout << "Received " << i << " packets." << endl;*/
#endif


		if (nTime > timestamps.nPingTimeout && isInitialized)
		{
			lock();
			isInitialized = false;
			AvlTreeIterator iterator(pAvlTree);
			AvlNode* pNode;
			while ((pNode = iterator.next()) != NULL)
			{
				if (pNode->pData != NULL)
				{
					LinkUpLabel* pLabel = ((LinkUpLabel*)pNode->pData);
					pLabel->isInitialized = false;
					pLabel->timestamps.nInitTryTimeout = 0;
					pList->insert(pLabel);
					pAvlTree->remove(pLabel->nIdentifier);
					if (pLabel->nType == LinkUpLabelType::Event)
					{
						((LinkUpEventLabel*)pLabel)->unsubscribed(NULL);
						pEventList->remove(pLabel);
					}
				}
			}
			unlock();
			uint8_t* pBuffer = new uint8_t[1024];
			while (getRaw(pBuffer, 1024) > 0)
			{
			}
		}

		if (!isInitialized && nTime > timestamps.nInitTryTimeout && pName != NULL)
		{
			timestamps.nInitTryTimeout = nTime + initialization_timeout;
			timestamps.nPingTimeout = nTime + ping_timeout;
			LinkUpPacket packet;
			packet.nLength = strlen(pName) + (uint16_t)sizeof(LinkUpLogic) + (uint16_t)sizeof(LinkUpNameRequest);
			packet.pData = (uint8_t*)calloc(packet.nLength, sizeof(uint8_t));

			LinkUpLogic* logic = (LinkUpLogic*)packet.pData;
			LinkUpNameRequest* nameRequest = (LinkUpNameRequest*)logic->pInnerHeader;

			logic->nLogicType = LinkUpLogicType::NameRequest;
			nameRequest->nLabelType = LinkUpLabelType::Node;
			nameRequest->nNameLength = strlen(pName);
			memcpy(nameRequest->pName, pName, nameRequest->nNameLength);

			connector.send(packet);
		}
	}
	if (isInitialized && nProgressType & LinkUpProgressType::Normal)
	{
		lock();
		LinkedListIterator iterator(pList);
		unlock();
		LinkUpLabel* pLabel;

		while ((pLabel = (LinkUpLabel*)iterator.next()) != NULL)
		{
			pLabel->progress(&connector);
		}

		LinkedListIterator eventIterator(pEventList);

		while ((pLabel = (LinkUpLabel*)eventIterator.next()) != NULL)
		{
			pLabel->progress(&connector);
		}
	}

}

LinkUpNode::LinkUpNode(const char* pName)
{
	this->pName = (char*)calloc(strlen(pName) + 1, sizeof(uint8_t));
	strcpy(this->pName, pName);
}

void LinkUpNode::receivedPacket(LinkUpPacket packet, uint32_t nTime)
{
	if (packet.nLength > 0 && packet.pData != NULL) {
		LinkUpLogic* logic = (LinkUpLogic*)packet.pData;
		switch (logic->nLogicType)
		{
		case LinkUpLogicType::NameRequest:
			receivedNameRequest(packet, (LinkUpNameRequest*)logic->pInnerHeader);
			break;
		case LinkUpLogicType::NameResponse:
			receivedNameResponse(packet, (LinkUpNameResponse*)logic->pInnerHeader);
			break;
		case LinkUpLogicType::PropertyGetRequest:
			receivedPropertyGetRequest(packet, (LinkUpPropertyGetRequest*)logic->pInnerHeader);
			break;
		case LinkUpLogicType::PropertyGetResponse:
			receivedPropertyGetResponse(packet, (LinkUpPropertyGetResponse*)logic->pInnerHeader);
			break;
		case LinkUpLogicType::PropertySetRequest:
			receivedPropertySetRequest(packet, (LinkUpPropertySetRequest*)logic->pInnerHeader);
			break;
		case LinkUpLogicType::PropertySetResponse:
			receivedPropertySetResponse(packet, (LinkUpPropertySetResponse*)logic->pInnerHeader);
			break;
		case LinkUpLogicType::PingRequest:
			receivedPingRequest(packet, nTime);
			break;
		case LinkUpLogicType::EventFireResponse:
			receivedEventFireResponse(packet, (LinkUpEventFireResponse*)logic->pInnerHeader);
			break;
		case LinkUpLogicType::EventSubscribeRequest:
			receivedEventSubscribeRequest(packet, (LinkUpEventSubscribeRequest*)logic->pInnerHeader);
			break;
		case LinkUpLogicType::EventUnsubscribeRequest:
			receivedEventUnsubscribeRequest(packet, (LinkUpEventUnsubscribeRequest*)logic->pInnerHeader);
			break;
		case LinkUpLogicType::FunctionCallRequest:
			receivedFunctionCallRequest(packet, (LinkUpFunctionCallRequest*)logic->pInnerHeader);
			break;
		default:
			break;
		}
	}

	if (packet.pData != NULL) {
		free(packet.pData);
		packet.pData = NULL;
	}
}

void LinkUpNode::receivedNameRequest(LinkUpPacket packet, LinkUpNameRequest* pNameRequest) {
}

void LinkUpNode::receivedNameResponse(LinkUpPacket packet, LinkUpNameResponse* pNameResponse) {
	char *pResponseName;
	uint32_t nLength = pNameResponse->nNameLength;

	pResponseName = (char*)calloc(nLength + 1, sizeof(uint8_t));
	memcpy(pResponseName, pNameResponse->pName, nLength);

	if (pNameResponse->nLabelType == LinkUpLabelType::Node && strcmp(pName, pResponseName) == 0)
	{
		nIdentifier = pNameResponse->nIdentifier;
		isInitialized = true;
	}
	else
	{
		lock();
		LinkedListIterator iterator(pList);
		LinkUpLabel* pLabel;

		while ((pLabel = (LinkUpLabel*)iterator.next()) != NULL)
		{
			if (pLabel->receivedNameResponse(pResponseName, pNameResponse->nLabelType, pNameResponse->nIdentifier))
			{
				pAvlTree->insert(pNameResponse->nIdentifier, pLabel);
				pList->remove(pLabel);
			}
		}
		unlock();
	}
	free(pResponseName);
}

void LinkUpNode::receivedPropertyGetRequest(LinkUpPacket packet, LinkUpPropertyGetRequest* pPropertyGetRequest)
{
	lock();
	if (pAvlTree != NULL && pPropertyGetRequest != NULL) {
		AvlNode* pNode = pAvlTree->find(pPropertyGetRequest->nIdentifier);
		if (pNode != NULL && pNode->pData != NULL) {
			LinkUpLabel* label = (LinkUpLabel*)pNode->pData;
			if (label->nType == LinkUpLabelType::Property) {
				((LinkUpPropertyLabel*)label)->receivedPropertyGetRequest(&connector);
			}
		}
		else {
			//TODO: error??
		}
	}
	unlock();
}

void LinkUpNode::receivedEventFireResponse(LinkUpPacket packet, LinkUpEventFireResponse* pEventFireResponse)
{
	//TODO:
}

void LinkUpNode::receivedEventSubscribeRequest(LinkUpPacket packet, LinkUpEventSubscribeRequest* pEventSubscribeRequest)
{
	lock();
	if (pAvlTree != NULL && pEventSubscribeRequest != NULL) {
		AvlNode* pNode = pAvlTree->find(pEventSubscribeRequest->nIdentifier);
		if (pNode != NULL && pNode->pData != NULL) {
			LinkUpLabel* label = (LinkUpLabel*)pNode->pData;
			if (label->nType == LinkUpLabelType::Event) {
				((LinkUpEventLabel*)label)->subscribed(&connector);
				pEventList->insert(label);
			}
		}
	}
	unlock();
}

void LinkUpNode::receivedEventUnsubscribeRequest(LinkUpPacket packet, LinkUpEventUnsubscribeRequest* pEventUnsubscribeRequest)
{
	lock();
	if (pAvlTree != NULL && pEventUnsubscribeRequest != NULL) {
		AvlNode* pNode = pAvlTree->find(pEventUnsubscribeRequest->nIdentifier);
		if (pNode != NULL && pNode->pData != NULL) {
			LinkUpLabel* label = (LinkUpLabel*)pNode->pData;
			if (label->nType == LinkUpLabelType::Event) {
				((LinkUpEventLabel*)label)->unsubscribed(&connector);
				pEventList->remove(label);
			}
		}
	}
	unlock();
}

void LinkUpNode::receivedPropertyGetResponse(LinkUpPacket packet, LinkUpPropertyGetResponse* pPropertyGetResponse)
{

}

void LinkUpNode::receivedFunctionCallRequest(LinkUpPacket packet, LinkUpFunctionCallRequest* pFunctionCallRequest)
{
	lock();
	if (pAvlTree != NULL && pFunctionCallRequest != NULL) {
		AvlNode* pNode = pAvlTree->find(pFunctionCallRequest->nIdentifier);
		if (pNode != NULL && pNode->pData != NULL) {
			LinkUpLabel* label = (LinkUpLabel*)pNode->pData;
			if (label->nType == LinkUpLabelType::Function) {
				((LinkUpFunctionLabel*)label)->receivedFunctionCallRequest(pFunctionCallRequest->pData, packet.nLength - sizeof(LinkUpLogic) - sizeof(LinkUpFunctionCallRequest), &connector);
			}
		}
	}
	unlock();
}

void LinkUpNode::receivedPropertySetRequest(LinkUpPacket packet, LinkUpPropertySetRequest* pPropertySetRequest)
{
	lock();
	if (pAvlTree != NULL && pPropertySetRequest != NULL) {
		AvlNode* pNode = pAvlTree->find(pPropertySetRequest->nIdentifier);
		if (pNode != NULL && pNode->pData != NULL) {
			LinkUpLabel* label = (LinkUpLabel*)pNode->pData;
			if (label->nType == LinkUpLabelType::Property) {
				((LinkUpPropertyLabel*)label)->receivedPropertySetRequest(pPropertySetRequest->pData, packet.nLength - sizeof(LinkUpLogic) - sizeof(LinkUpPropertySetRequest), &connector);
			}
		}
		else {
			//TODO: error??
		}
	}
	unlock();
}

void LinkUpNode::receivedPropertySetResponse(LinkUpPacket packet, LinkUpPropertySetResponse* pPropertySetResponse) {
}

void LinkUpNode::receivedPingRequest(LinkUpPacket packet, uint32_t nTime)
{
	LinkUpPacket response;
	response.nLength = sizeof(LinkUpLogic);
	response.pData = (uint8_t*)calloc(response.nLength, sizeof(uint8_t));

	LinkUpLogic* logic = (LinkUpLogic*)response.pData;
	LinkUpPropertyGetResponse* getResponse = (LinkUpPropertyGetResponse*)logic->pInnerHeader;

	logic->nLogicType = LinkUpLogicType::PingResponse;

	timestamps.nPingTimeout = nTime + ping_timeout;

	connector.send(response);
}

void LinkUpNode::addLabel(LinkUpLabel* pLabel)
{
	lock();
	pList->insert(pLabel);
	if (pLabel->nType == LinkUpLabelType::Function)
	{
		pEventList->insert(pLabel);
	}
	unlock();
}