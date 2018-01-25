#include "CoreCommandBuffer.h"
#include "CoreConfig.h"
//--------------------------------------------------------------------------------------------------------------------------------------
CoreCommandBuffer Commands(&Serial);
//--------------------------------------------------------------------------------------------------------------------------------------
CoreCommandBuffer::CoreCommandBuffer(Stream* s) : pStream(s)
{
    strBuff = new String();
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreCommandBuffer::hasCommand()
{
  if(!(pStream && pStream->available()))
    return false;

    char ch;
    while(pStream->available())
    {
      ch = pStream->read();
      if(ch == '\r' || ch == '\n')
      {
        return strBuff->length() > 0; // вдруг лишние управляющие символы придут в начале строки?
      } // if

      *strBuff += ch;
      // не даём вычитать больше символов, чем надо - иначе нас можно заспамить
      if(strBuff->length() >= 200)
      {
         clearCommand();
         return false;
      } // if
    } // while

    return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CommandParser::CommandParser()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CommandParser::~CommandParser()
{
  clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CommandParser::clear()
{
  for(size_t i=0;i<arguments.size();i++)
  {
    delete [] arguments[i];  
  }
  
  while(arguments.size())
    arguments.pop();
}
//--------------------------------------------------------------------------------------------------------------------------------------
const char* CommandParser::getArg(size_t idx) const
{
  if(arguments.size() && idx < arguments.size())
    return arguments[idx];

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CommandParser::parse(const String& command, bool isSetCommand)
{
  clear();
    // разбиваем на аргументы
    const char* startPtr = command.c_str() + strlen_P(isSetCommand ? (const char* )CORE_COMMAND_SET : (const char*) CORE_COMMAND_GET);
    size_t len = 0;

    while(*startPtr)
    {
      const char* delimPtr = strchr(startPtr,'|');
            
      if(!delimPtr)
      {
        len = strlen(startPtr);
        char* newArg = new char[len + 1];
        memset(newArg,0,len+1);
        strncpy(newArg,startPtr,len);
        arguments.push_back(newArg);        

        return arguments.size();
      } // if(!delimPtr)

      size_t len = delimPtr - startPtr;

     
      char* newArg = new char[len + 1];
      memset(newArg,0,len+1);
      strncpy(newArg,startPtr,len);
      arguments.push_back(newArg);

      startPtr = delimPtr + 1;
      
    } // while      

  return arguments.size();
    
}
//--------------------------------------------------------------------------------------------------------------------------------------


