/**
   @file
   @author Shin'ichiro Nakaoka
*/

#include "MultiValueSeqItem.h"
#include "MultiSeqItemCreationPanel.h"
#include "ItemManager.h"
#include <boost/bind.hpp>
#include "gettext.h"

using namespace cnoid;

static bool loadPlainSeqFormat(MultiValueSeqItem* item, const std::string& filename, std::ostream& os)
{
    if(item->seq()->loadPlainFormat(filename)){
        return true;
    } else {
        os << item->seq()->seqMessage();
        return false;
    }
}


static bool saveAsPlainSeqFormat(MultiValueSeqItem* item, const std::string& filename, std::ostream& os)
{
    if(item->seq()->saveAsPlainFormat(filename)){
        return true;
    } else {
        os << item->seq()->seqMessage();
        return false;
    }
}


template<> void MultiSeqItem<MultiValueSeq>::initializeClass(ExtensionManager* ext)
{
    ext->itemManager().registerClass<MultiValueSeqItem>(N_("MultiValueSeqItem"));

    ext->itemManager().addCreationPanel<MultiValueSeqItem>(
        new MultiSeqItemCreationPanel(_("Number of values in a frame")));
    
    ext->itemManager().addLoaderAndSaver<MultiValueSeqItem>(
        _("Plain Format of a Multi Value Sequence"), "PLAIN-MULTI-VALUE-SEQ", "*",
        boost::bind(loadPlainSeqFormat, _1, _2, _3), boost::bind(saveAsPlainSeqFormat, _1, _2, _3), 
        ItemManager::PRIORITY_CONVERSION);
}

#ifdef WIN32
template class MultiSeqItem<MultiValueSeq>;
#endif
