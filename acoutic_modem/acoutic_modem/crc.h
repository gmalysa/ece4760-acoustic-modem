void CRC8_InitChecksum( unsigned char &crcvalue );
void CRC8_UpdateChecksum( unsigned char &crcvalue, const void *data, int length );
void CRC8_FinishChecksum( unsigned char &crcvalue );
unsigned char CRC8_BlockChecksum( const void *data, int length );