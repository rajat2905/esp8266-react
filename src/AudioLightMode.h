#ifndef AudioLightMode_h
#define AudioLightMode_h

#include <LedSettingsService.h>
#include <FrequencySampler.h>

#define AUDIO_LIGHT_MODE_FILE_PATH_PREFIX "/modes/"
#define AUDIO_LIGHT_MODE_FILE_PATH_SUFFIX ".json"
#define AUDIO_LIGHT_MODE_SERVICE_PATH_PREFIX "/rest/modes/"

class AudioLightMode {
 public:
  virtual const String& getId() = 0;
  virtual void begin() = 0;
  virtual void readFromFS() = 0;
  virtual void writeToFS() = 0;
  virtual void tick() = 0;
  virtual void enable() = 0;
  virtual void sampleComplete(){};
};

template <class T>
class AudioLightModeImpl : public StatefulService<T>, public AudioLightMode {
 protected:
  String _id;
  LedSettingsService* _ledSettingsService;
  FrequencySampler* _frequencySampler;
  HttpEndpoint<T> _httpEndpoint;
  FSPersistence<T> _fsPersistence;

 public:
  AudioLightModeImpl(AsyncWebServer* server,
                 FS* fs,
                 SecurityManager* securityManager,
                 LedSettingsService* ledSettingsService,
                 FrequencySampler* frequencySampler,
                 JsonStateReader<T> stateReader,
                 JsonStateUpdater<T> stateUpdater,
                 const String& id) :
      _id(id),
      _ledSettingsService(ledSettingsService),
      _frequencySampler(frequencySampler),
      _httpEndpoint(stateReader, stateUpdater, this, server, AUDIO_LIGHT_MODE_SERVICE_PATH_PREFIX + id, securityManager),
      _fsPersistence(stateReader,
                     stateUpdater,
                     this,
                     fs,
                     String(AUDIO_LIGHT_MODE_FILE_PATH_PREFIX + id + AUDIO_LIGHT_MODE_FILE_PATH_SUFFIX).c_str()) {
  }

  /*
   * Get the code for the mode as a string
   */
  const String& getId() {
    return _id;
  }
  /*
   * Read the config from the file system and disable the update handler
   */
  void begin() {
    _fsPersistence.disableUpdateHandler();
    _fsPersistence.readFromFS();
  }

  /*
   * Load the mode config from the file system
   */
  void readFromFS() {
    _fsPersistence.readFromFS();
  }

  /*
   * Save the mode config to the file system
   */
  void writeToFS() {
    _fsPersistence.writeToFS();
  }
};

#endif  // end AudioLightMode_h
