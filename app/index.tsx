import { Card } from "@/components/ui/card";
import { Heading } from "@/components/ui/heading";
import { Text } from "@/components/ui/text";
import { useRouter } from "expo-router";
import React, { useEffect, useRef, useState } from "react";
import { FlatList, Pressable, View, Platform } from "react-native";
// Native voice support
let NativeVoice: any = null;
try {
  // react-native-voice is a native module — require it at runtime so web bundlers don't choke
  // it will only be present after `npm install` and rebuilding a dev client or native app.
  // eslint-disable-next-line @typescript-eslint/no-var-requires
  NativeVoice = require("@react-native-voice/voice");
} catch (e) {
  NativeVoice = null;
}
import { Mic as MicIcon } from "lucide-react-native";

const mockTasks = [
  { id: "1", title: "Prepare Medicine", description: "Prepare and dispense medication" },
  { id: "2", title: "Set up the table", description: "Arrange plates, cups and utensils" },
  { id: "3", title: "Organize Books", description: "Sort and place books on the shelf" },
];
//changed to include the speech button
export default function Home() {
  const router = useRouter();
  const [listening, setListening] = useState(false);
  const [transcript, setTranscript] = useState("");
  const recognitionRef = useRef<any | null>(null);
  const nativeListeningRef = useRef(false);

  useEffect(() => {
    // cleanup on unmount
    return () => {
      // web cleanup
      if (recognitionRef.current) {
        try {
          recognitionRef.current.onresult = null;
          recognitionRef.current.onend = null;
          recognitionRef.current.onerror = null;
          recognitionRef.current.stop();
        } catch (e) {
          // ignore
        }
      }

      // native cleanup
      if (NativeVoice) {
        try {
          NativeVoice.destroy();
          NativeVoice.removeAllListeners && NativeVoice.removeAllListeners();
        } catch (e) {
          // ignore
        }
      }
    };
  }, []);

  const handleMapping = (text: string) => {
    const t = text.toLowerCase();

    // Simple phrase matching to map speech to task routes.
    if (t.includes("medicine") || t.includes("prepare medicine") || t.includes("prepare the medicine")) {
      router.push(`/task/1`);
      return;
    }

    if (t.includes("set up") || t.includes("set the table") || t.includes("set up the table") || t.includes("table")) {
      router.push(`/task/2`);
      return;
    }

    if (t.includes("organize") || t.includes("organise") || t.includes("books") || t.includes("organize books") || t.includes("organize bookshelf") || t.includes("bookshelf")) {
      router.push(`/task/3`);
      return;
    }

    // fallback: go to task screen without id
    router.push(`/task`);
  };

  const startListening = () => {
    // If native voice is available (react-native-voice) and we're on a native platform, use it
    if (NativeVoice && Platform.OS !== "web") {
      try {
        // set up listeners
        NativeVoice.onSpeechResults = (event: any) => {
          const text = (event?.value && event.value[0]) || "";
          setTranscript(text);
          handleMapping(text);
        };
        NativeVoice.onSpeechError = (event: any) => {
          console.error("Native Voice error", event);
          setListening(false);
          nativeListeningRef.current = false;
        };

        NativeVoice.start("en-US");
        setListening(true);
        nativeListeningRef.current = true;
      } catch (e) {
        console.error("Failed to start native voice", e);
        alert("Failed to start native speech recognition. Make sure you rebuilt the native app with the native module.");
      }

      return;
    }

    // Fallback: web SpeechRecognition
    if (typeof window === "undefined") {
      alert("Speech recognition is only supported in the web or native builds of this app.");
      return;
    }

    const SpeechRecognition = (window as any).SpeechRecognition || (window as any).webkitSpeechRecognition;
    if (!SpeechRecognition) {
      alert("Speech recognition is not supported in this browser.");
      return;
    }

    try {
      const recognition = new SpeechRecognition();
      recognition.lang = "en-US";
      recognition.interimResults = false;
      recognition.maxAlternatives = 1;

      recognition.onresult = (event: any) => {
        const text = event.results[0][0].transcript;
        setTranscript(text);
        handleMapping(text);
      };

      recognition.onerror = (event: any) => {
        console.error("Speech recognition error", event);
        setListening(false);
      };

      recognition.onend = () => {
        setListening(false);
      };

      recognitionRef.current = recognition;
      setListening(true);
      recognition.start();
    } catch (e) {
      console.error(e);
      alert("Failed to start speech recognition.");
    }
  };

  const stopListening = () => {
    // stop web
    if (recognitionRef.current) {
      try {
        recognitionRef.current.stop();
      } catch (e) {
        // ignore
      }
      recognitionRef.current = null;
    }

    // stop native
    if (NativeVoice && nativeListeningRef.current) {
      try {
        NativeVoice.stop();
      } catch (e) {
        // ignore
      }
      nativeListeningRef.current = false;
    }

    setListening(false);
  };

  return (
    <View className="py-safe flex-1 bg-gray-50 px-5">
      <Heading size="2xl" className="mb-6">
        ADL Robot
      </Heading>

      <Heading className="mb-4" size="xl">
        Available Tasks
      </Heading>
      <FlatList
        data={mockTasks}
        keyExtractor={(item) => item.id}
        ItemSeparatorComponent={() => <View className="h-3" />}
        renderItem={({ item }) => (
          <Pressable
            onPress={() => {
              router.push(`/task/${item.id}`);
            }}
            accessibilityRole="button"
            style={{ marginBottom: 8 }}
          >
            <Card variant="outline" className="text-center">
              <Heading>{item.title}</Heading>
              <Text>{item.description}</Text>
            </Card>
          </Pressable>
        )}
      />

      <View className="mt-6 items-center">
        <Pressable
          onPress={() => (listening ? stopListening() : startListening())}
          accessibilityRole="button"
          accessibilityLabel={listening ? "Stop voice listening" : "Start voice listening"}
          className={`w-32 h-32 rounded-full items-center justify-center ${listening ? "bg-red-500" : "bg-blue-600"}`}
        >
          <MicIcon color="white" size={48} />
        </Pressable>

        <Text className="mt-3 text-base text-center">{listening ? "Listening... Tap to stop" : "Speak a command"}</Text>

        {transcript ? (
          <Text className="mt-2 text-sm text-center">Heard: {transcript}</Text>
        ) : null}
      </View>
    </View>
  );
}
